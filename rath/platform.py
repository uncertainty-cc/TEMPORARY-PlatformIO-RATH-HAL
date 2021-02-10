from platformio.managers.platform import PlatformBase

class RathPlatform(PlatformBase):
    def get_boards(self, board_id=None):
        result = PlatformBase.get_boards(self, board_id)
        if not result:
            return result
        if board_id:
            return self._add_dynamic_options(result)
        else:
            for key, value in result.items():
                result[key] = self._add_dynamic_options(result[key])
            return result

    def _add_dynamic_options(self, board):
        # upload protocols
        if not board.get("upload.protocols", []):
            board.manifest['upload']['protocols'] = ["serial"]
        if not board.get("upload.protocol", ""):
            board.manifest['upload']['protocol'] = "serial"

        # debug tools
        debug = board.manifest.get("debug", {})
        non_debug_protocols = ["serial"]
        supported_debug_tools = [
            "rv-link",
            "ra-link",
            "sipeed-rv-debugger"
        ]

        upload_protocol = board.manifest.get("upload", {}).get("protocol")
        upload_protocols = board.manifest.get("upload", {}).get("protocols", [])
        upload_protocols.extend(supported_debug_tools)
        if upload_protocol and upload_protocol not in upload_protocols:
            upload_protocols.append(upload_protocol)
        board.manifest['upload']['protocols'] = upload_protocols

        if "tools" not in debug:
            debug['tools'] = {}

        # Only FTDI based debug probes
        for link in upload_protocols:
            if link == "rv-link":
                debug['tools']['rv-link'] = {
                    "hwids": [["0x28e9", "0x018a"]],
                    "require_debug_port": True,
                    "init_cmds": [
                        "define pio_reset_halt_target",
                        "    monitor reset halt",
                        "end",
                        "define pio_reset_target",
                        "    monitor reset",
                        "end",
                        "target extended-remote $DEBUG_PORT",
                        "$INIT_BREAK",
                        "pio_reset_halt_target",
                        "$LOAD_CMDS",
                        "monitor init",
                        "pio_reset_halt_target"
                    ]
                }
            else:
                openocd_interface = "ftdi/" + link
                
                if link == "ra-link":
                    openocd_interface = "ftdi/sipeed-rv-debugger"

                server_args = [
                    "-s", "$PACKAGE_DIR/share/openocd/scripts",
                    "-f", "interface/%s.cfg" % openocd_interface,
                    "-c", "transport select jtag",
                    "-f", "target/gd32vf103.cfg"
                ]
                server_args.append("-c")

                server_args.append("adapter_khz 1000")

                if link != "rv-link":
                    debug['tools'][link] = {
                        "server": {
                            "package": "tool-openocd-gd32v",
                            "executable": "bin/openocd",
                            "arguments": server_args
                        }
                    }

        board.manifest['debug'] = debug
        return board
