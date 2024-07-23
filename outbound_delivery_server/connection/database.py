import os
import mysql.connector
from configparser import ConfigParser

def path():
    current_path = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_path)
    
    return current_dir

def connection():
    config = ConfigParser()
    current_dir = path()
    config.read(current_dir + "/config.ini")
    
    conn = mysql.connector.connect(host=config["database"]["host"],
                                   user=config["database"]["user"],
                                   port=config["database"]["port"],
                                   password=config["database"]["password"],
                                   database=config["database"]["database"])
    return conn
