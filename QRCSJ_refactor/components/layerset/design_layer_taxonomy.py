# postpones evaluation of type annotations
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping
from types import SimpleNamespace



DESIGN_LAYER_TYPE_TAXONOMY = {
    "utility": {
        "optical": {},
        "ebeam": {
            "small_writefield": {},
            "large_writefield": {},
        },
        "general": {},
    },
    "exposure": {
        "optical": {},
        "ebeam": {
            "devices": {},
            "dose_test": {},
        },
        "ebeam_large": {},
    },
}
"""
Tree structure representing the available `DesignLayerType` options.

Taxononomy: the scientific process of classifying things (= arranging them into groups).
"""

### Current taxonomy tree:

# ├── utility
# │   ├── optical
# │   ├── ebeam
# │   │   ├── small_writefield
# │   │   └── large_writefield
# │   └── general
# └── exposure
#     ├── optical
#     ├── ebeam
#     │   ├── devices
#     │   └── dose_test
#     └── ebeam_large


@dataclass(frozen=True, order=True)
class DesignLayerType:
    parts: tuple[str, ...]

    @classmethod
    def of(cls, *parts: str) -> DesignLayerType:
        return cls(tuple(norm_token(p) for p in parts))

    @classmethod
    def from_str(cls, s: str, sep: str = "/") -> DesignLayerType:
        return cls(tuple(norm_token(p) for p in s.split(sep) if p.strip()))

    def __str__(self) -> str:
        return "/".join(self.parts)
    
    def is_under(self, prefix: "DesignLayerType") -> bool:
        return self.parts[: len(prefix.parts)] == prefix.parts


def norm_token(s: str) -> str:
    """
    Normalizes tokens to be used with `DesignLayerType` taxonomy.

    Example
    -------
    "Small writefield" -> "small_writefield"
    """
    
    return "_".join(s.strip().lower().split())


def validate_leaf_type(t: DesignLayerType, taxonomy: Mapping[str, Any]) -> None:
    """
    Validates whether `t` is a leaf in the `taxonomy` tree.
    """
    node: Any = taxonomy
    for part in t.parts:
        if not isinstance(node, Mapping) or part not in node:
            raise ValueError(f"Invalid layer type '{t}'. Unknown token '{part}'.")
        node = node[part]

    if not isinstance(node, Mapping):
        raise ValueError(f"Invalid taxonomy structure at '{t}' (expected dict).")

    if len(node) != 0:
        # It's an internal node, not a leaf
        children = ", ".join(sorted(node.keys()))
        raise ValueError(
            f"Layer type '{t}' is not a leaf. Choose one of: {children}"
        )
    

def list_leaf_types(taxonomy: Mapping[str, Any] = DESIGN_LAYER_TYPE_TAXONOMY) -> list[DesignLayerType]:
    """Return all leaf DesignLayerTypes in the `taxonomy`."""
    out: list[DesignLayerType] = []

    def _recurse(node: Mapping[str, Any], prefix: tuple[str, ...]) -> None:
        for k, v in node.items():
            p = prefix + (k,)
            if isinstance(v, Mapping) and len(v) == 0:
                out.append(DesignLayerType(p))
            elif isinstance(v, Mapping):
                _recurse(v, p)

    _recurse(taxonomy, ())
    return out


def print_taxonomy(taxonomy: dict, prefix: str = "") -> None:
    """
    Pretty-print a taxonomy tree.

    Leaves are printed as final nodes.
    """
    items = list(taxonomy.items())
    for i, (key, subtree) in enumerate(items):
        last = i == len(items) - 1
        branch = "└── " if last else "├── "
        print(prefix + branch + key)

        if isinstance(subtree, dict) and subtree:
            next_prefix = prefix + ("    " if last else "│   ")
            print_taxonomy(subtree, next_prefix)