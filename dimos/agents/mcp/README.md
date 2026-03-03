# DimOS MCP Server

Expose DimOS robot skills via Model Context Protocol. Supports OpenClaw and Claude Code as clients.

## Setup

```bash
uv sync --extra base --extra unitree
```

Start DimOS with MCP:

```bash
uv run dimos run unitree-go2-agentic-mcp
```

This starts the MCP server on `http://localhost:9990/mcp`.

## OpenClaw

The OpenClaw plugin lives at `dimos/web/plugin_openclaw/`. It bridges DimOS MCP tools into the OpenClaw agent system.

From the plugin directory:

```bash
cd dimos/web/plugin_openclaw
pnpm install
# set ANTHROPIC_API_KEY and OPENCLAW_GATEWAY_TOKEN in .env
pnpm openclaw plugins install -l .
pnpm openclaw config set plugins.entries.dimos.enabled true
pnpm openclaw config set gateway.mode local
```

Run the gateway (with DimOS MCP already running):

```bash
pnpm openclaw gateway run --port 18789 --verbose
```

You should see `dimos: discovered 13 tool(s)` confirming the plugin loaded.

Send commands:

```bash
pnpm openclaw agent --session-id dimos-test --message "move forward 1 meter"
```

Or use the interactive TUI:

```bash
pnpm openclaw tui
```

## Claude Code

Add the MCP server:

```bash
claude mcp add --transport http --scope project dimos http://localhost:9990/mcp
```

Verify:

```bash
claude mcp list
```

Use robot skills:

```
> move forward 1 meter
> go to the kitchen
> tag this location as "desk"
```

## MCP Inspector

For manual inspection, use MCP Inspector:

```bash
npx -y @modelcontextprotocol/inspector
```

Change **Transport Type** to "Streamable HTTP", **URL** to `http://localhost:9990/mcp`, and **Connection Type** to "Direct". Click "Connect".

## How It Works

1. `McpServer` in the blueprint starts a FastAPI server on port 9990
2. Clients (Claude Code, OpenClaw) connect to `http://localhost:9990/mcp`
3. Skills are exposed as MCP tools (e.g., `relative_move`, `navigate_with_text`)
