# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time

import dimos.core as core
from dimos.vr.modules import MetaQuestModule

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_metaquest_module():
    dimos = core.start(1)

    try:
        quest = dimos.deploy(MetaQuestModule, host="0.0.0.0", port=8881)

        logger.info("Generating SSL certificate...")
        cert_result = quest.generate_certificate().result()
        logger.info(f"Certificate result: {cert_result}")

        logger.info("Starting VR server...")
        quest.start().result()

        stats = quest.get_stats().result()
        logger.info(f"Server stats: {stats}")

        logger.info("VR server running. Access at: https://localhost:8881")
        logger.info("Connect with Quest 3 to test controller streaming")

        time.sleep(30)

        final_stats = quest.get_stats().result()
        logger.info(f"Final stats: {final_stats}")

        quest.stop().result()
        logger.info("VR server stopped")

    finally:
        dimos.shutdown()


if __name__ == "__main__":
    test_metaquest_module()
