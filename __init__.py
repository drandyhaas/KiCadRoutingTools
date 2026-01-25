# KiCad Routing Tools - SWIG Action Plugin Registration
# This file must be at the root of the plugin directory for KiCad to find it

import os
import sys

# Add this directory to Python path so kicad_routing_plugin can be found
_plugin_dir = os.path.dirname(os.path.abspath(__file__))
if _plugin_dir not in sys.path:
    sys.path.insert(0, _plugin_dir)

try:
    from kicad_routing_plugin.action_plugin import KiCadRoutingToolsPlugin
    KiCadRoutingToolsPlugin().register()
except Exception as e:
    import logging
    logging.getLogger("KiCadRoutingTools").error(f"Failed to register plugin: {e}")
