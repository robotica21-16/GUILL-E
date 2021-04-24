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
