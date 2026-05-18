#!/usr/bin/env bash
# ==============================================================
#  AgriBot – Full System Startup Script (Backend + Frontend)
#  Usage:  ./start.sh
#
#  Runs the WHOLE system:
#    - Backend  : FastAPI/uvicorn on http://0.0.0.0:8000
#    - Frontend : static Next.js export on http://0.0.0.0:3000
#
#  Backend only?  ->  cd Backend && ./start.sh
# ==============================================================

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$ROOT/Backend"
FRONTEND_DIR="$ROOT/Frontend/out"

SERVER_PID=""
FRONTEND_PID=""
TUNNEL_PID=""

# ------------------------------------------------------------------
# Cleanup on exit (Ctrl+C or script end)
# ------------------------------------------------------------------
cleanup() {
    echo ""
    echo "🛑 Shutting down AgriBot (full system)…"

    if [[ -n "$TUNNEL_PID" ]] && kill -0 "$TUNNEL_PID" 2>/dev/null; then
        echo "   Stopping Cloudflare tunnel (PID $TUNNEL_PID)…"
        kill "$TUNNEL_PID" 2>/dev/null
        wait "$TUNNEL_PID" 2>/dev/null
    fi

    if [[ -n "$SERVER_PID" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
        echo "   Stopping uvicorn backend (PID $SERVER_PID)…"
        kill "$SERVER_PID" 2>/dev/null
        wait "$SERVER_PID" 2>/dev/null
    fi

    if [[ -n "$FRONTEND_PID" ]] && kill -0 "$FRONTEND_PID" 2>/dev/null; then
        echo "   Stopping frontend server (PID $FRONTEND_PID)…"
        kill "$FRONTEND_PID" 2>/dev/null
        wait "$FRONTEND_PID" 2>/dev/null
    fi

    echo "✅ AgriBot stopped. Goodbye!"
    exit 0
}

trap cleanup SIGINT SIGTERM

# ------------------------------------------------------------------
# Banner
# ------------------------------------------------------------------
echo "╔══════════════════════════════════════════╗"
echo "║      🌱  AgriBot — Full System  🤖       ║"
echo "╠══════════════════════════════════════════╣"
echo "║   Backend (API)  +  Frontend (Web UI)    ║"
echo "╚══════════════════════════════════════════╝"
echo ""

# ------------------------------------------------------------------
# Backend — virtual environment
# ------------------------------------------------------------------
cd "$BACKEND_DIR" || { echo "❌ Backend dir not found: $BACKEND_DIR"; exit 1; }

if [[ ! -d "venv" ]]; then
    echo "⚠️  No virtual environment found. Creating one…"
    python3 -m venv venv
fi

source ./venv/bin/activate
echo "✅ Virtual environment activated ($(python3 --version))"

# ------------------------------------------------------------------
# Backend — install dependencies (with retry)
# ------------------------------------------------------------------
install_deps() {
    echo "📦 Installing dependencies…"
    for attempt in 1 2 3; do
        if pip install -r requirements.txt --timeout 30; then
            echo "✅ Dependencies installed"
            return 0
        fi
        echo "⚠️  Install failed (attempt $attempt/3). Retrying in 5s…"
        sleep 5
    done
    echo "❌ Could not install dependencies after 3 attempts."
    echo "   Check your internet connection and try again."
    exit 1
}

# Only install if uvicorn is missing (skip on subsequent runs)
if ! python3 -c "import uvicorn" 2>/dev/null; then
    install_deps
fi
echo ""

# ------------------------------------------------------------------
# Start the FastAPI backend
# ------------------------------------------------------------------
echo "🚀 Starting AgriBot API server…"
python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 &
SERVER_PID=$!

# Verify the process actually started
sleep 1
if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "❌ Backend failed to start. Check errors above."
    exit 1
fi

# Wait for backend to be ready
echo -n "   Waiting for backend"
for i in {1..15}; do
    if curl -s http://localhost:8000/ > /dev/null 2>&1; then
        echo ""
        echo "✅ Backend live at http://0.0.0.0:8000"
        echo "📄 API docs at    http://0.0.0.0:8000/docs"
        break
    fi
    echo -n "."
    sleep 1
done
echo ""

# ------------------------------------------------------------------
# Start the Frontend (prebuilt static export)
# ------------------------------------------------------------------
if [[ -f "$FRONTEND_DIR/index.html" ]]; then
    echo "🌐 Starting frontend (static) …"
    python3 -m http.server 3000 --directory "$FRONTEND_DIR" > /dev/null 2>&1 &
    FRONTEND_PID=$!

    sleep 1
    if ! kill -0 "$FRONTEND_PID" 2>/dev/null; then
        echo "⚠️  Frontend failed to start (port 3000 in use?). Backend keeps running."
        FRONTEND_PID=""
    else
        echo -n "   Waiting for frontend"
        for i in {1..10}; do
            if curl -s http://localhost:3000/ > /dev/null 2>&1; then
                echo ""
                echo "✅ Frontend live at http://0.0.0.0:3000"
                break
            fi
            echo -n "."
            sleep 1
        done
        echo ""
    fi
else
    echo "⚠️  Frontend not found at $FRONTEND_DIR/index.html — skipping (backend only)."
fi
echo ""

# ------------------------------------------------------------------
# Cloudflare tunnel prompt (15 s timeout → defaults to No)
# ------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🌐 Cloudflare Tunnel lets you access the AgriBot"
echo "   backend from anywhere on the internet."
echo ""

TUNNEL_CHOICE=""
read -t 15 -p "   Start Cloudflare tunnel for the backend? (y/N, auto-skip in 15s): " TUNNEL_CHOICE || true
echo ""

if [[ "$TUNNEL_CHOICE" == "y" || "$TUNNEL_CHOICE" == "Y" ]]; then
    if command -v cloudflared &> /dev/null; then
        echo "🔗 Starting Cloudflare tunnel…"
        cloudflared tunnel --url http://localhost:8000 &
        TUNNEL_PID=$!
        sleep 3
        echo "✅ Tunnel is running (PID $TUNNEL_PID)"
    else
        echo "⚠️  cloudflared not found. Install it with:"
        echo "   curl -L https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64 -o /usr/local/bin/cloudflared"
        echo "   chmod +x /usr/local/bin/cloudflared"
    fi
else
    echo "⏭️  Skipping Cloudflare tunnel (local network only)"
fi

# ------------------------------------------------------------------
# Keep running
# ------------------------------------------------------------------
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🟢 AgriBot full system is running. Press Ctrl+C to stop."
echo "   Backend : http://0.0.0.0:8000"
[[ -n "$FRONTEND_PID" ]] && echo "   Frontend: http://0.0.0.0:3000"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Wait for the background processes; trap handles Ctrl+C → graceful shutdown
wait
