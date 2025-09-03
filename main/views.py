from django.shortcuts import render
import yaml
import os
from django.conf import settings
from django.http import JsonResponse, HttpResponseBadRequest
import threading
from yasmin_msgs.msg import StateMachine
import rclpy
from std_msgs.msg import Bool
from django.views.decorators.http import require_GET

class ROS2NodeManager:
    _instance = None
    _lock = threading.Lock()

    def __init__(self):
        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node('snaak_ui')
        self.publisher = self.node.create_publisher(Bool, 'snaak_ui/start_recipe', 10)
        self.fsm_state = None
        self.node.create_subscription(
            StateMachine,
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
        fsm_viewer_states = msg.states
        if len(fsm_viewer_states) > 0:
            self.fsm_state = fsm_viewer_states[fsm_viewer_states[0].current_state].name
        else:
            self.fsm_state = None

    @classmethod
    def get_instance(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = ROS2NodeManager()
            return cls._instance

    def publish_start_recipe(self):
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)
    
def user_page(request):
    # Set your YAML file paths here
    stock_path = '/home/snaak/Documents/recipe/stock.yaml'
    recipe_path = '/home/snaak/Documents/recipe/recipe.yaml'
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
    })

def operator_page(request):
    return render(request, 'operator_page.html')

@require_GET
def fsm_state_api(request):
    state = ROS2NodeManager.get_instance().fsm_state
    print(state)
    return JsonResponse({'fsm_state': state})
