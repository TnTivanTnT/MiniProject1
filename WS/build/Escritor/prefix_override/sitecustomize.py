import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ivan/Escritorio/MiniProyects/1/GH/WS/install/Escritor'
