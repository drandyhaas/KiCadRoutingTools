#!/usr/bin/env python3
"""Modal vLLM server: the LLM half of the cloud plan+route service.

    modal deploy tests/stress/modal_route/vllm_app.py

Serves an OpenAI-compatible endpoint that the routing agent (route_app.py)
points opencode at. Deployed (not `modal run`) so it persists and SCALES TO
ZERO between uses -- you pay GPU-seconds only while an agent session is
actually talking to it (plus the scaledown window).

Model choice (KICAD_LLM_MODEL at deploy time):
  default  Qwen/Qwen3-Coder-30B-A3B-Instruct -- agentic/tool-calling MoE that
           fits ONE 80 GB GPU: ~$2-4/h while hot, well under $5 per board.
  big      moonshotai/Kimi-K2-Instruct -- 1T-param MoE (32B active), needs
           KICAD_LLM_GPU="H100!:8" and ~$30-50/h while hot. The quality
           option when a board is worth it; same endpoint contract.

The weights land in a persistent volume on first start (tens of minutes for
K2, a few minutes for the default); later cold starts are loading, not
downloading.

Auth: vLLM's --api-key, from the `kicad-llm-token` Modal secret
(`modal secret create kicad-llm-token KICAD_LLM_TOKEN=<random>`). The
endpoint URL is public; the token is what gates it.
"""
from __future__ import annotations

import os
import subprocess

import modal

MODEL = os.environ.get("KICAD_LLM_MODEL", "Qwen/Qwen3-Coder-30B-A3B-Instruct")
GPU = os.environ.get("KICAD_LLM_GPU", "A100-80GB")
SERVED_NAME = "kicad-router-llm"   # stable name the agent config uses
PORT = 8000

app = modal.App("kicad-route-llm")

hf_cache = modal.Volume.from_name("kicad-hf-cache", create_if_missing=True)

image = (
    modal.Image.debian_slim(python_version="3.12")
    .pip_install("vllm>=0.10.0", "huggingface_hub[hf_transfer]>=0.26")
    .env({"HF_HUB_ENABLE_HF_TRANSFER": "1", "KICAD_LLM_MODEL": MODEL})
)


@app.function(
    image=image,
    gpu=GPU,
    volumes={"/root/.cache/huggingface": hf_cache},
    timeout=6 * 3600,
    scaledown_window=300,   # idle GPU minutes you are willing to pay after a session
    secrets=[modal.Secret.from_name("kicad-llm-token")],
)
@modal.concurrent(max_inputs=32)
@modal.web_server(port=PORT, startup_timeout=45 * 60)
def serve():
    model = os.environ.get("KICAD_LLM_MODEL", MODEL)
    ngpu = int(os.environ.get("KICAD_LLM_TP", "0")) or (
        int(GPU.split(":")[1]) if ":" in GPU else 1)
    cmd = [
        "vllm", "serve", model,
        "--host", "0.0.0.0", "--port", str(PORT),
        "--served-model-name", SERVED_NAME,
        "--tensor-parallel-size", str(ngpu),
        "--api-key", os.environ["KICAD_LLM_TOKEN"],
        # agentic use: tool calling on, generous context
        "--enable-auto-tool-choice", "--tool-call-parser", "hermes",
    ]
    subprocess.Popen(cmd)
