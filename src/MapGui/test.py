import sys
from PyQt5.QtCore import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWidgets import QApplication

sys.argv.append("--disable-web-security") 
app = QApplication(sys.argv)

path = QDir.current().filePath('map.js')
local = QUrl.fromLocalFile(path).toString()

html = '<html><head>'
html += '<link rel="stylesheet" href="https://unpkg.com/leaflet@1.6.0/dist/leaflet.css" integrity="sha512-xwE/Az9zrjBIphAcBb3F6JVqxf46+CDLwfLMHloNu6KEQCAWi6HcDUbeOfBIptF7tcCzusKFjFw2yuvEpDL9wQ==" crossorigin=""/>'
html += '<script src="https://unpkg.com/leaflet@1.6.0/dist/leaflet.js" integrity="sha512-gZwIG9x3wUXg2hdXF6+rVkLF/0Vi9U8D2Ntg4Ga5I5BZpVkVxlJWbSQtXPSiUTtC0TjtGOmxa1AJPuV0CPthew==" crossorigin=""></script>'
html += '<style> body { padding: 0; margin: 0; } #mapid { height: 400px;} </style>'
html += '</head><body><div id="mapid"></div>'
html += '<script src="{}"></script>'.format(local)
html += '</body></html>'

web = QWebEngineView()
web.setHtml(html)
web.show()
 
sys.exit(app.exec_())
