#Need to install pywebview before running
#install with "pip install pywebview"

#run server.py before running this file

import webview

webview.create_window ('RSX Map Software', 'http://localhost:8080/index.html')
webview.start(gui='qt')