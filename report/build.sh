#!/bin/sh
# Copies the latest figures/tables from ../results and compiles the report twice
# (twice so that the Index and the Lists show correct page numbers).
cd "$(dirname "$0")"
mkdir -p figures tables
cp ../results/figures/*.png figures/
cp ../results/tables/*.tex tables/
pdflatex -interaction=nonstopmode main.tex > /dev/null
pdflatex -interaction=nonstopmode main.tex > /dev/null
grep -E "^! " main.log && exit 1
echo "built $(pwd)/main.pdf ($(pdfinfo main.pdf 2>/dev/null | grep Pages))"
