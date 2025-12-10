# 🔐 Security & Safety

## ⚠️ Network Warning
> [!WARNING]
> This application binds to `0.0.0.0` by default.

- **Risk**: The UI is accessible to anyone on your local network.
- **Action**: Use only on trusted Wi-Fi (Home/Office) or behind a VPN. Do not expose port `5000` to the internet.

## 🔑 API Keys
- **Storage**: Keys are loaded from `.env` only.
- **Best Practice**: Never commit `.env` to git.

## 🛑 Physical Safety
- **Emergency Stop**: The UI "STOP" button halts the software loop immediately.
- **Hardware Kill Switch**: Recommended for larger motors/robots.

## 🙈 Privacy
- **Processing**: Video streams are processed **locally**.
- **AI Data**: Only discrete frame snapshots are sent to OpenAI while the AI is active.
