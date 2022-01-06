import os
import sqlite3
import subprocess
from dataclasses import dataclass

@dataclass
class LineFindItem:
    image_filepath: str
    timestamp_src: str
    timestamp_start_pos: int
    timestamp_format: str
    calib_filepath: str
    result_image_filepath: str = ""
    
    def __init__(self, img_path, tm_stamp_src, tm_stamp_start_pos, tm_stamp_fmt, cal_path, result_img_path = ""):
        self.image_filepath = img_path
        self.timestamp_src = tm_stamp_src
        self.timestamp_start_pos = tm_stamp_start_pos
        self.timestamp_format = tm_stamp_fmt
        self.calib_filepath = cal_path
        self.result_image_filepath = result_img_path

class Images():
    grime2cli_path = None
    conn = None
    db_filepath = None
    cursor = None
    
    def __init__(self, grime2cli_filepath = "bin/grime2cli"):
        self.grime2cli_path = grime2cli_filepath
            
    def open_db(self, db_path):
        self.conn = sqlite3.connect(db_path)
        self.db_filepath = db_path
        self.cursor = self.conn.cursor()
        
    def close_db(self):
        if None is self.conn:
            print('No database open')
        else:
            self.conn.close()
            self.db_filepath = ""
            cursor = None
            
    def add_image(self, img_path):
        if None is self.cursor:
            print('No database open to add image')
        else:
            sql_str = "INSERT INTO images VALUES (\'"
            sql_str += img_path + "\')";
            print(sql_str)
            self.cursor.execute(sql_str)
            
    def get_image_filepaths(self, image_top_level_folderpath):
        image_paths = []
        for root, dirs, files in os.walk(image_top_level_folderpath, topdown=True):
            for name in files:
                filename, file_ext = os.path.splitext(name)
                fext_lower = file_ext.lower()
                if fext_lower == '.png' or fext_lower == '.jpg':
                    image_paths.append(os.path.join(root, name))
        return image_paths
    
    def find_line(self, item):
        timestamp_type = '--timestamp_from_filename'
        if 'filename' != item.timestamp_src:
            timestamp_type = '--timestamp_from_exif'
        result = subprocess.run([self.grime2cli_path, '--find_line', item.image_filepath, timestamp_type,
                                 '--timestamp_start_pos', str(item.timestamp_start_pos), '--timestamp_format', item.timestamp_format,
                                 '--calib_json', item.calib_filepath, '--result_image', item.result_image_filepath], stdout=subprocess.PIPE)
        return result.stdout


                    
    def init_images_db(self, image_top_level_folderpath):
        if not os.path.isdir(image_top_level_folderpath):
            print(self.db_filepath + "Invalid top level folderpath")
        elif self.cursor is not None:
            self.conn.close();
            os.remove(self.db_filepath)
            self.open_db(self.db_filepath)
            print("Database created successfully")
            # print(self.db_filepath + "Init image db failed. No database open")
            self.cursor.execute("CREATE TABLE images (filepath TEXT, result TEXT)")
            print("Table created successfully")
            image_paths = self.get_image_filepaths(image_top_level_folderpath)
            if 0 == len(image_paths):
                print('No names found in folder ' + image_top_level_folderpath)
            else:
                for img in image_paths:
                    # print(img)
                    self.add_image(img)
                self.conn.commit()

if __name__ == "__main__":
    image_access = Images()
    item = LineFindItem('/home/pi/Projects/GRIME2/gcgui/config/2012_demo/06/NRmarshDN-12-06-30-10-30.jpg',
                        'filename', 10, 'yy-mm-dd-hh-mm', '/home/pi/Projects/GRIME2/gcgui/config/calib.json',
                        '/var/tmp/water/find_line_result.png')
    output = image_access.find_line(item)
    print(output.decode())
    
# if __name__ == "__main__":
#     image_access = Images()
#     image_access.open_db('/var/tmp/water/ncmarsh.db')
#     image_access.init_images_db('/media/kchapman/Elements/data/NCMarch_for_article_two/2012_02_ncmarsh_raw_images/')
