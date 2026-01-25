"""
Routing Exceptions

Exception classes for routing operations, allowing the router to report
errors through exceptions instead of sys.exit() calls.
"""


class RoutingError(Exception):
    """Base exception for all routing errors."""
    pass


class RoutingCancelled(RoutingError):
    """Raised when routing is cancelled by the user."""
    pass


class ConfigurationError(RoutingError):
    """Raised when the routing configuration is invalid."""
    pass


class NoPathFoundError(RoutingError):
    """Raised when no valid path can be found for a net."""

    def __init__(self, net_id: int, net_name: str = "", message: str = ""):
        self.net_id = net_id
        self.net_name = net_name
        super().__init__(message or f"No path found for net {net_name or net_id}")


class ObstacleMapError(RoutingError):
    """Raised when obstacle map construction fails."""
    pass


class InputFileError(RoutingError):
    """Raised when input file cannot be read or parsed."""
    pass


class OutputFileError(RoutingError):
    """Raised when output file cannot be written."""
    pass


class LayerError(RoutingError):
    """Raised when layer configuration is invalid."""
    pass


class GridResolutionError(RoutingError):
    """Raised when grid resolution causes issues."""
    pass
