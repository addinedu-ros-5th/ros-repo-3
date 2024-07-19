from flask import Flask
from router.manager import manager
from router.reader import reader
from router.led import led

app = Flask(__name__)

app.register_blueprint(manager, url_prefix='/manager')
app.register_blueprint(reader, url_prefix='/tag')
app.register_blueprint(led, url_prefix='/led')

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)