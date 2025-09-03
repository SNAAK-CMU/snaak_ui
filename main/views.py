from django.shortcuts import render
import yaml
import os
from django.conf import settings
from django.http import JsonResponse, HttpResponseBadRequest
import threading

class ROS2NodeManager:
    _instance = None
    _lock = threading.Lock()

    def __init__(self):
        import rclpy
        from std_msgs.msg import Bool, String
        self.rclpy = rclpy
        self.Bool = Bool
        self.String = String
        rclpy.init(args=None)
        self.node = rclpy.create_node('django_node')
        self.publisher = self.node.create_publisher(Bool, 'start_recipe', 10)
        self.fsm_viewer_data = None
        self.node.create_subscription(
            String,
            '/fsm_viewer',
            self.fsm_viewer_callback,
            10
        )
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        while self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def fsm_viewer_callback(self, msg):
        self.fsm_viewer_data = msg.data

    @classmethod
    def get_instance(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = ROS2NodeManager()
            return cls._instance

    def publish_start_recipe(self):
        msg = self.Bool()
        msg.data = True
        self.publisher.publish(msg)
    
    def get_fsm_current_state(self):
        try:
            if self.fsm_viewer_data:
                data = self.fsm_viewer_data
                if len(data) > 0:
                    state = data[data[0].current_state].name
                    return state
        except:
            return None

def user_page(request):
    # Set your YAML file paths here
    stock_path = 'stock.yaml'
    recipe_path = 'recipes.yaml'
    try:
        with open(stock_path) as f:
            stock = yaml.safe_load(f)
    except Exception as e:
        return render(request, 'user_page.html', {'error': f'Could not load stock file: {e}'})
    try:
        with open(recipe_path) as f:
            recipe_data = yaml.safe_load(f)
    except Exception as e:
        recipe_data = {}
    # Flatten recipe to dict
    recipe = {}
    if isinstance(recipe_data, dict) and 'recipe' in recipe_data:
        for entry in recipe_data['recipe']:
            recipe.update(entry)
    # New: stock['ingredients'] is a dict
    ingredients = []
    for name, info in stock.get('ingredients', {}).items():
        if name.lower() == 'bread':
            continue
        ingredients.append({'name': name, 'available': info.get('slices', 0)})
    bread_default = 2
    # Get FSM state from ROS2
    fsm_state = ROS2NodeManager.get_instance().get_fsm_current_state()
    state_is_recipe = (fsm_state == "recipe")

    # post signifies that form/button submitted on page
    if request.method == "POST": # only one button so do not have to differentiate actions
        try:
            data = yaml.safe_load(request.body) if request.body else request.POST
            # Enforce stock limits and bread default
            new_recipe = []
            new_recipe.append({'bread': bread_default})
            for ing in ingredients:
                name = ing['name']
                max_qty = ing['available']
                qty = min(int(data.get(name, 0)), max_qty)
                if qty > 0:
                    new_recipe.append({name: qty})
            with open(recipe_path, 'w') as f:
                yaml.dump({'recipe': new_recipe}, f)
            ROS2NodeManager.get_instance().publish_start_recipe()
            return JsonResponse({"status": "success"})
        except Exception as e:
            return HttpResponseBadRequest(str(e))
    return render(request, 'user_page.html', { # update fields used by html
        'ingredients': ingredients,
        'recipe': recipe,
        'bread_default': bread_default,
        'stock_path': stock_path,
        'recipe_path': recipe_path,
        'fsm_state': fsm_state,
    })

def operator_page(request):
    return render(request, 'operator_page.html')
