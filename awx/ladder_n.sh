#!/bin/bash
# The braid stage over the ladder on EXISTING fanout boards (the
# chain's ch_fo_k*.kicad_pcb), so braid changes are compared on
# identical inputs. usage: ladder_n.sh TAG K [K...]   (FO=prefix)
cd "$(dirname "$0")"
TAG=$1
shift
FO=${FO:-ch}
for K in "$@"; do
  echo "=== K$K $(date +%H:%M:%S)"
  bash braid_k.sh "${FO}_fo_k${K}.kicad_pcb" "$TAG" "$K"
done
echo "=== ladder done $(date +%H:%M:%S)"
