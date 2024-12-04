import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/edu-bat/Documentos/GitHub/modulo-8/atividade_2_slam/meu_workspace/install/ola_mundo'
