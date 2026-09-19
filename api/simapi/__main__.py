import logging

import uvicorn

from . import config

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
uvicorn.run("simapi.app:app", host=config.API_HOST, port=config.API_PORT, log_level="info",
            timeout_keep_alive=120)  # long keep-alive: RL clients issue ~100 req/s per simulator
