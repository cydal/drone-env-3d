import logging

import uvicorn

from . import config

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
uvicorn.run("simapi.app:app", host=config.API_HOST, port=config.API_PORT, log_level="info")
