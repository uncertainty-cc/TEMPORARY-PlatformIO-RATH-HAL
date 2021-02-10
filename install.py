import os
import shutil
from pathlib import Path
import logging

""" config logging """
logger = logging.getLogger("Installer")
logger.setLevel(logging.DEBUG)

ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(logging.Formatter("%(asctime)s %(levelname)s: [%(name)s]: %(message)s"))

logger.addHandler(ch)

del ch


""" main program """
framework_folder_name = "framework-rath-hal"
platform_folder_name = "rath"

home = str(Path.home())

pio_home = Path(home, ".platformio")

framework_path = Path(pio_home, "packages", framework_folder_name)
platform_path = Path(pio_home, "platforms", platform_folder_name)

try:
    logger.info("deleting framework path...")
    shutil.rmtree(framework_path)
except FileNotFoundError:
    logger.info("no framework folder found. pass.")
    pass
try:
    logger.info("deleting platform path...")
    shutil.rmtree(platform_path)
except FileNotFoundError:
    logger.info("no platform folder found. pass.")
    pass

logger.info("copying framework path...")
shutil.copytree(Path(Path.cwd(), framework_folder_name), framework_path)
logger.info("copying platform path...")
shutil.copytree(Path(Path.cwd(), platform_folder_name), platform_path)


logger.info("Finshed.")
os.system("pause")
