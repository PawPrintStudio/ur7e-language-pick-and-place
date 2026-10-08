#!/usr/bin/env bash
# Rebuild docs/diagrams/ros2_node_graph.pdf from the *.dot sources.
# Needs Graphviz (`dot`, in the devcontainer) and one PDF joiner:
# pdfunite (poppler-utils), qpdf, ghostscript, or python3 + pypdf.
set -euo pipefail
cd "$(dirname "$0")"
pages=()
for src in [0-9][0-9]_*.dot; do
  out="${src%.dot}.pdf"
  dot -Tpdf "$src" -o "$out"
  pages+=("$out")
done
target=ros2_node_graph.pdf
if command -v pdfunite >/dev/null; then
  pdfunite "${pages[@]}" "$target"
elif command -v qpdf >/dev/null; then
  qpdf --empty --pages "${pages[@]}" -- "$target"
elif command -v gs >/dev/null; then
  gs -q -dNOPAUSE -dBATCH -sDEVICE=pdfwrite -sOutputFile="$target" "${pages[@]}"
else
  python3 - "$target" "${pages[@]}" <<'EOF'
import sys
from pypdf import PdfWriter
writer = PdfWriter()
for page in sys.argv[2:]:
    writer.append(page)
writer.write(sys.argv[1])
EOF
fi
rm -f "${pages[@]}"
echo "wrote docs/diagrams/$target"
