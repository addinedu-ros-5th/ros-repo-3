from flask import Flask
from router.manager import manager
from router.reader import reader
from router.led import led
from router.camera import camera

app = Flask(__name__)

app.register_blueprint(manager, url_prefix='/manager')
app.register_blueprint(reader, url_prefix='/tag')
app.register_blueprint(led, url_prefix='/led')
app.register_blueprint(camera, url_prefix='/camera')

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)