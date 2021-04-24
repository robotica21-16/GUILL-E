#!/bin/bash

# uso:
# - plot del ultimo log de la carpeta logs:
#     bash plotLog.sh
# - plot de otro log:
#     bash plotLog.sh log/loquesea.txt

if [ $# -eq 0 ]; then
    file="logs/$(ls logs -Art | tail -n 1)"
else
    file=$1
fi
echo "----------- plotting file $file --------------"
sed -i '$d' "$file"
sed -i '$d' "$file"
sed -i '$d' "$file"
python p4/plotp4.py -m trabajo/mapaA_CARRERA.txt -l "$file"
