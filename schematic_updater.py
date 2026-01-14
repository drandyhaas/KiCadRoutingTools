"""
Schematic Updater - Updates KiCad schematic files with pad swaps from routing.

When routing applies pad swaps (target swaps or polarity swaps), this module
updates the corresponding .kicad_sch files to keep schematics in sync with PCB.
"""

import os
import re
from typing import List, Dict, Optional, Tuple


def find_schematic_files(schematic_dir: str) -> List[str]:
    """Find all .kicad_sch files in a directory (non-recursive)."""
    if not os.path.isdir(schematic_dir):
        return []

    sch_files = []
    for filename in os.listdir(schematic_dir):
        if filename.endswith('.kicad_sch'):
            sch_files.append(os.path.join(schematic_dir, filename))
    return sch_files


def find_all_schematics_for_component(schematic_dir: str, component_ref: str) -> List[str]:
    """
    Find all .kicad_sch files containing a component reference.

    Multi-unit symbols (like U2A, U2B) may have different units in different
    schematic files, but the lib_symbol definition is embedded in each file.
    This returns ALL files with a matching Reference so the caller can try each.

    Args:
        schematic_dir: Directory containing .kicad_sch files
        component_ref: Component reference designator (e.g., "U3", "IC1")

    Returns:
        List of paths to schematic files (may be empty)
    """
    sch_files = find_schematic_files(schematic_dir)
    matching_files = []

    # Pattern to find symbol instances with matching Reference property
    # Look for: (property "Reference" "U3" ...) within a (symbol ...) block
    ref_pattern = re.compile(
        r'\(property\s+"Reference"\s+"' + re.escape(component_ref) + r'"',
        re.IGNORECASE
    )

    for sch_path in sch_files:
        try:
            with open(sch_path, 'r', encoding='utf-8') as f:
                content = f.read()

            if ref_pattern.search(content):
                matching_files.append(sch_path)
        except (IOError, UnicodeDecodeError):
            continue

    return matching_files


def swap_pins_in_schematic(schematic_path: str, component_ref: str,
                           pad1: str, pad2: str, verbose: bool = False) -> bool:
    """
    Swap two pin numbers for a component in a schematic file.

    This swaps the (number "...") values in the lib_symbols section,
    which changes which physical pin each signal is assigned to.

    Args:
        schematic_path: Path to .kicad_sch file
        component_ref: Component reference (e.g., "U3")
        pad1: First pin number to swap (e.g., "C10")
        pad2: Second pin number to swap (e.g., "E10")
        verbose: Print debug info

    Returns:
        True if swap was successful, False otherwise
    """
    try:
        with open(schematic_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except (IOError, UnicodeDecodeError) as e:
        if verbose:
            print(f"    Error reading {schematic_path}: {e}")
        return False

    # First, find which lib_symbol this component uses by finding a symbol instance
    # Symbol instances look like: (symbol (lib_id "Library:SymbolName") ... (property "Reference" "U3") ...)

    # Find symbol instance with matching Reference
    symbol_instance_pattern = re.compile(
        r'\(symbol\s*\n?\s*\(lib_id\s+"([^"]+)"\)',
        re.DOTALL
    )

    lib_id = None
    for match in symbol_instance_pattern.finditer(content):
        # Find the end of this symbol block
        start_pos = match.start()
        depth = 0
        end_pos = start_pos
        for i, c in enumerate(content[start_pos:]):
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    end_pos = start_pos + i + 1
                    break

        symbol_block = content[start_pos:end_pos]

        # Check if this symbol has our Reference
        ref_match = re.search(
            r'\(property\s+"Reference"\s+"' + re.escape(component_ref) + r'"',
            symbol_block
        )
        if ref_match:
            lib_id = match.group(1)
            break

    if not lib_id:
        if verbose:
            print(f"    Component {component_ref} not found in {schematic_path}")
        return False

    # Now find the lib_symbols section and the symbol definition for this lib_id
    # lib_symbols format: (lib_symbols (symbol "Library:SymbolName" ...pin definitions...))

    # The symbol name in lib_symbols matches the lib_id
    lib_symbol_pattern = re.compile(
        r'\(symbol\s+"' + re.escape(lib_id) + r'"',
        re.DOTALL
    )

    lib_symbol_match = lib_symbol_pattern.search(content)
    if not lib_symbol_match:
        if verbose:
            print(f"    lib_symbol for {lib_id} not found")
        return False

    # Find the full lib_symbol block
    start_pos = lib_symbol_match.start()
    depth = 0
    end_pos = start_pos
    for i, c in enumerate(content[start_pos:]):
        if c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                end_pos = start_pos + i + 1
                break

    lib_symbol_block = content[start_pos:end_pos]

    # Find the pin definitions with (number "pad1") and (number "pad2")
    # Pin number format (spans multiple lines):
    # (number "E21"
    #     (effects ...)
    # )
    num1_pattern = re.compile(r'\(number\s+"' + re.escape(pad1) + r'"', re.DOTALL)
    num2_pattern = re.compile(r'\(number\s+"' + re.escape(pad2) + r'"', re.DOTALL)

    num1_matches = list(num1_pattern.finditer(lib_symbol_block))
    num2_matches = list(num2_pattern.finditer(lib_symbol_block))

    if not num1_matches or not num2_matches:
        if verbose:
            if not num1_matches:
                print(f"    Pin number {pad1} not found in lib_symbol")
            if not num2_matches:
                print(f"    Pin number {pad2} not found in lib_symbol")
        return False

    # Swap all occurrences of the pin numbers
    # Use placeholders to avoid double-replacement
    placeholder1 = f"__PIN_NUM_PLACEHOLDER_1_{pad1}__"
    placeholder2 = f"__PIN_NUM_PLACEHOLDER_2_{pad2}__"

    new_lib_symbol_block = lib_symbol_block

    # Replace pad1 with placeholder
    new_lib_symbol_block = re.sub(
        r'\(number\s+"' + re.escape(pad1) + r'"',
        f'(number "{placeholder1}"',
        new_lib_symbol_block
    )

    # Replace pad2 with placeholder
    new_lib_symbol_block = re.sub(
        r'\(number\s+"' + re.escape(pad2) + r'"',
        f'(number "{placeholder2}"',
        new_lib_symbol_block
    )

    # Now replace placeholders with swapped values
    new_lib_symbol_block = new_lib_symbol_block.replace(
        f'(number "{placeholder1}"',
        f'(number "{pad2}"'
    )
    new_lib_symbol_block = new_lib_symbol_block.replace(
        f'(number "{placeholder2}"',
        f'(number "{pad1}"'
    )

    # Replace the lib_symbol block in the content
    new_content = content[:start_pos] + new_lib_symbol_block + content[end_pos:]

    # Write the modified content back
    try:
        with open(schematic_path, 'w', encoding='utf-8') as f:
            f.write(new_content)
        return True
    except IOError as e:
        if verbose:
            print(f"    Error writing {schematic_path}: {e}")
        return False


def apply_swaps_to_schematics(schematic_dir: str, swap_list: List[Dict],
                               verbose: bool = False) -> Tuple[int, int]:
    """
    Apply all swaps to schematic files.

    Args:
        schematic_dir: Directory containing .kicad_sch files
        swap_list: List of swap dicts with keys:
            - component_ref: str (e.g., "U3")
            - pad1: str (e.g., "C10")
            - pad2: str (e.g., "E10")
        verbose: Print progress info

    Returns:
        (swaps_applied, swaps_failed) tuple
    """
    if not swap_list:
        return (0, 0)

    if not os.path.isdir(schematic_dir):
        print(f"Warning: Schematic directory not found: {schematic_dir}")
        return (0, len(swap_list))

    print(f"\nUpdating schematics in {schematic_dir}...")

    swaps_applied = 0
    swaps_failed = 0

    # Group swaps by component to find schematics efficiently
    component_swaps: Dict[str, List[Dict]] = {}
    for swap in swap_list:
        comp = swap['component_ref']
        if comp not in component_swaps:
            component_swaps[comp] = []
        component_swaps[comp].append(swap)

    # Cache of component -> list of candidate schematic files
    component_to_schematics: Dict[str, List[str]] = {}

    for component_ref, swaps in component_swaps.items():
        # Find all schematic files that contain this component
        if component_ref not in component_to_schematics:
            component_to_schematics[component_ref] = find_all_schematics_for_component(
                schematic_dir, component_ref
            )

        candidate_files = component_to_schematics[component_ref]

        if not candidate_files:
            print(f"  Warning: Component {component_ref} not found in any schematic")
            swaps_failed += len(swaps)
            continue

        for swap in swaps:
            pad1 = swap['pad1']
            pad2 = swap['pad2']

            # Update ALL candidate files that have the lib_symbol with these pins
            # (the same lib_symbol definition is embedded in every schematic file
            # that uses any unit of this component)
            files_updated = []
            for sch_path in candidate_files:
                if swap_pins_in_schematic(sch_path, component_ref, pad1, pad2, verbose=verbose):
                    files_updated.append(os.path.basename(sch_path))

            if files_updated:
                print(f"  {component_ref}: {pad1} <-> {pad2} in {', '.join(files_updated)}")
                swaps_applied += 1
            else:
                print(f"  Warning: Failed to swap {component_ref}:{pad1} <-> {pad2}")
                swaps_failed += 1

    return (swaps_applied, swaps_failed)


def collect_swaps_from_state(state) -> List[Dict]:
    """
    Collect all swaps from a RoutingState into a unified format.

    Args:
        state: RoutingState object from routing

    Returns:
        List of swap dicts with {component_ref, pad1, pad2}
    """
    swaps = []

    # Single-ended target swaps
    if hasattr(state, 'single_ended_target_swap_info'):
        for info in state.single_ended_target_swap_info:
            if 'n1_pad' in info and 'n2_pad' in info:
                pad1 = info['n1_pad']
                pad2 = info['n2_pad']
                # Only add if same component (typical case)
                if pad1.component_ref == pad2.component_ref:
                    swaps.append({
                        'component_ref': pad1.component_ref,
                        'pad1': pad1.pad_number,
                        'pad2': pad2.pad_number
                    })
                else:
                    # Different components - add both
                    swaps.append({
                        'component_ref': pad1.component_ref,
                        'pad1': pad1.pad_number,
                        'pad2': pad2.pad_number
                    })
                    swaps.append({
                        'component_ref': pad2.component_ref,
                        'pad1': pad2.pad_number,
                        'pad2': pad1.pad_number
                    })

    # Diff pair target swaps
    if hasattr(state, 'target_swap_info'):
        for info in state.target_swap_info:
            # Each diff pair swap involves 4 pads: p1_p, p1_n, p2_p, p2_n
            # The P pads swap with each other, and N pads swap with each other
            if all(k in info for k in ['p1_p_pad', 'p2_p_pad', 'p1_n_pad', 'p2_n_pad']):
                p1_p = info['p1_p_pad']
                p2_p = info['p2_p_pad']
                p1_n = info['p1_n_pad']
                p2_n = info['p2_n_pad']

                # P pads swap
                if p1_p.component_ref == p2_p.component_ref:
                    swaps.append({
                        'component_ref': p1_p.component_ref,
                        'pad1': p1_p.pad_number,
                        'pad2': p2_p.pad_number
                    })

                # N pads swap
                if p1_n.component_ref == p2_n.component_ref:
                    swaps.append({
                        'component_ref': p1_n.component_ref,
                        'pad1': p1_n.pad_number,
                        'pad2': p2_n.pad_number
                    })

    # Polarity swaps (uses pad_p/pad_n keys)
    if hasattr(state, 'pad_swaps'):
        for swap_info in state.pad_swaps:
            if 'pad_p' in swap_info and 'pad_n' in swap_info:
                pad_p = swap_info['pad_p']
                pad_n = swap_info['pad_n']
                if pad_p.component_ref == pad_n.component_ref:
                    swaps.append({
                        'component_ref': pad_p.component_ref,
                        'pad1': pad_p.pad_number,
                        'pad2': pad_n.pad_number
                    })

    return swaps
