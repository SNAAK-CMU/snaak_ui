from django.apps import AppConfig
import os, sys

class MainConfig(AppConfig):
    default_auto_field = 'django.db.models.BigAutoField'
    name = 'main'

    def ready(self):
        # only start ROS node in main process, in case Django starts multiple processes        
        if os.environ.get('RUN_MAIN') == 'true' or (len(sys.argv) > 1 and sys.argv[1] != 'runserver'): 
            from .views import ROS2NodeManager
            ROS2NodeManager.get_instance()
