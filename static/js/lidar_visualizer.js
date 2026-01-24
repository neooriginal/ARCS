/**
 * LidarVisualizer
 * Renders 360° LIDAR scan data as a smooth filled polygon.
 */
class LidarVisualizer {
    constructor(canvasId, options = {}) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas ? this.canvas.getContext('2d') : null;
        this.maxDist = options.maxDist || 400; // cm
    }

    /**
     * Draw the radar visualization.
     * @param {Object} scanData - Sparse scan data {angle: distance}
     */
    draw(scanData) {
        if (!this.ctx || !this.canvas) return;

        const ctx = this.ctx;
        const w = this.canvas.width;
        const h = this.canvas.height;
        const cx = w / 2;
        const cy = h / 2;
        const maxRadius = Math.min(cx, cy) - 5;

        // Clear canvas
        ctx.clearRect(0, 0, w, h);

        // Draw range rings (1m intervals)
        ctx.strokeStyle = 'rgba(0, 255, 255, 0.15)';
        ctx.lineWidth = 1;
        for (let r = maxRadius / 4; r <= maxRadius; r += maxRadius / 4) {
            ctx.beginPath();
            ctx.arc(cx, cy, r, 0, Math.PI * 2);
            ctx.stroke();
        }

        // Draw crosshairs
        ctx.strokeStyle = 'rgba(0, 255, 255, 0.2)';
        ctx.beginPath();
        ctx.moveTo(cx, 0); ctx.lineTo(cx, h);
        ctx.moveTo(0, cy); ctx.lineTo(w, cy);
        ctx.stroke();

        // Draw scan polygon
        if (scanData && Object.keys(scanData).length > 0) {
            const angles = Object.keys(scanData).map(Number).sort((a, b) => a - b);

            ctx.beginPath();
            let first = true;
            for (const angle of angles) {
                const dist = Math.min(scanData[angle], this.maxDist);
                const radians = (angle - 90) * Math.PI / 180;
                const r = (dist / this.maxDist) * maxRadius;
                const x = cx + Math.cos(radians) * r;
                const y = cy + Math.sin(radians) * r;

                if (first) { ctx.moveTo(x, y); first = false; }
                else { ctx.lineTo(x, y); }
            }
            ctx.closePath();

            // Gradient fill (Red close -> Green far)
            const gradient = ctx.createRadialGradient(cx, cy, 0, cx, cy, maxRadius);
            gradient.addColorStop(0, 'rgba(255, 50, 50, 0.6)');
            gradient.addColorStop(0.2, 'rgba(255, 180, 0, 0.4)');
            gradient.addColorStop(0.5, 'rgba(0, 255, 100, 0.3)');
            gradient.addColorStop(1, 'rgba(0, 255, 100, 0.1)');
            ctx.fillStyle = gradient;
            ctx.fill();

            // Outline
            ctx.strokeStyle = 'rgba(0, 255, 200, 0.8)';
            ctx.lineWidth = 2;
            ctx.stroke();
        }

        // Draw robot center
        ctx.fillStyle = 'rgba(0, 200, 255, 1)';
        ctx.beginPath();
        ctx.arc(cx, cy, 4, 0, Math.PI * 2);
        ctx.fill();
    }
}
