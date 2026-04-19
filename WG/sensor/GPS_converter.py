import csv

def csv_to_kml(input_csv, output_kml):
    with open(input_csv, 'r') as csvfile, open(output_kml, 'w') as kmlfile:
        reader = csv.reader(csvfile)
        headers = next(reader)
        
        kmlfile.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        kmlfile.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        kmlfile.write('<Document>\n')
        
        for row in reader:
            latitude = row[0]  # 假設第一列是緯度
            longitude = row[1]  # 假設第二列是經度
            name = row[2] if len(row) > 2 else "Location"  # 假設第三列是名稱
            description = row[3] if len(row) > 3 else ""  # 假設第四列是描述
            
            kmlfile.write('  <Placemark>\n')
            kmlfile.write(f'    <name>{name}</name>\n')
            kmlfile.write(f'    <description>{description}</description>\n')
            kmlfile.write('    <Point>\n')
            kmlfile.write(f'      <coordinates>{longitude},{latitude},0</coordinates>\n')
            kmlfile.write('    </Point>\n')
            kmlfile.write('  </Placemark>\n')
        
        kmlfile.write('</Document>\n')
        kmlfile.write('</kml>\n')

# 使用示例
csv_to_kml('gps_data.csv', 'gps_data.kml')
