"""
Authentication Module
Provides password hashing, token generation, and route protection.
"""
import hashlib
import hmac
import secrets
import time
from functools import wraps
from flask import request, jsonify, redirect

from core.config_manager import config_manager, get_config

TOKEN_EXPIRY_DAYS = 30
SECRET_KEY = None


def _get_secret():
    """Get or generate the secret key for token signing."""
    global SECRET_KEY
    if SECRET_KEY:
        return SECRET_KEY
    
    key = get_config("AUTH_SECRET_KEY", "")
    if not key:
        key = secrets.token_hex(32)
        config_manager.set("AUTH_SECRET_KEY", key)
        config_manager._save()
    
    SECRET_KEY = key
    return key


def hash_password(password: str) -> str:
    """Hash password using SHA-256 with salt."""
    salt = secrets.token_hex(16)
    hash_obj = hashlib.pbkdf2_hmac('sha256', password.encode(), salt.encode(), 100000)
    return f"{salt}${hash_obj.hex()}"


def verify_password(password: str, stored_hash: str) -> bool:
    """Verify password against stored hash."""
    if not stored_hash or '$' not in stored_hash:
        return False
    
    salt, hash_hex = stored_hash.split('$', 1)
    hash_obj = hashlib.pbkdf2_hmac('sha256', password.encode(), salt.encode(), 100000)
    return hmac.compare_digest(hash_obj.hex(), hash_hex)


def generate_token(username: str) -> str:
    """Generate a signed authentication token."""
    secret = _get_secret()
    expiry = int(time.time()) + (TOKEN_EXPIRY_DAYS * 86400)
    payload = f"{username}:{expiry}"
    signature = hmac.new(secret.encode(), payload.encode(), hashlib.sha256).hexdigest()
    return f"{payload}:{signature}"


def verify_token(token: str) -> str | None:
    """Verify token and return username if valid, None otherwise."""
    if not token:
        return None
    
    try:
        parts = token.split(':')
        if len(parts) != 3:
            return None
        
        username, expiry_str, signature = parts
        expiry = int(expiry_str)
        
        if time.time() > expiry:
            return None
        
        secret = _get_secret()
        payload = f"{username}:{expiry_str}"
        expected_sig = hmac.new(secret.encode(), payload.encode(), hashlib.sha256).hexdigest()
        
        if hmac.compare_digest(signature, expected_sig):
            return username
        return None
    except Exception:
        return None


def is_auth_configured() -> bool:
    """Check if authentication has been set up."""
    return bool(get_config("AUTH_PASSWORD_HASH", ""))


def require_auth(f):
    """Decorator to protect routes with authentication."""
    @wraps(f)
    def decorated(*args, **kwargs):
        if not is_auth_configured():
            return f(*args, **kwargs)
        
        auth_header = request.headers.get('Authorization', '')
        token = None
        
        if auth_header.startswith('Bearer '):
            token = auth_header[7:]
        
        if not token:
            token = request.cookies.get('auth_token')
        
        username = verify_token(token)
        if not username:
            if request.path.startswith('/api/'):
                return jsonify({'error': 'Unauthorized'}), 401
            return redirect('/login')
        
        return f(*args, **kwargs)
    return decorated
