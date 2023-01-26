
for i in ur lr ll ul; do
    nohup ./LKH ./image_${i}.par > image_${i}.log 2>&1 &
done
