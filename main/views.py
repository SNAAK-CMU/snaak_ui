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
from snaak_weight_read.srv import ReadWeight
import glob

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
        self.node.weight_assembly_client = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_assembly/read_weight"
        )
        self.node.weight_bins_client = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_bins/read_weight"
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

    def get_weight(self):
        read_weight = ReadWeight.Request()
        future = self.node.weight_assembly_client.call_async(read_weight)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        assembly_weight = result.weight.data

        read_weight = ReadWeight.Request()
        future = self.node.weight_bins_client.call_async(read_weight)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        bin_weight = result.weight.data

        return (assembly_weight, bin_weight)
    
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
    # Find all *_check_image.jpg files in static/segmentation
    image_dir = os.path.join(settings.STATIC_ROOT or os.path.join(settings.BASE_DIR, 'main/static'), 'segmentation')
    image_pattern = os.path.join(image_dir, '*_check_image.jpg')
    images = glob.glob(image_pattern)
    # Get just the filenames for static serving
    image_files = [os.path.basename(img) for img in images]
    return render(request, 'operator_page.html', {'ingredient_images': image_files})

@require_GET
def stock_api(request):
    stock_path = '/home/snaak/Documents/recipe/stock.yaml'
    try:
        with open(stock_path) as f:
            stock = yaml.safe_load(f)
        # Extract ingredient stock
        ingredients = stock.get('ingredients', {})
        # Format: {name: available}
        stock_data = {name: info.get('slices', 0) for name, info in ingredients.items()}
        return JsonResponse({'stock': stock_data})
    except Exception as e:
        return JsonResponse({'error': str(e)}, status=500)

@require_GET
def weight_api(request):
    assembly_weight, bin_weight = ROS2NodeManager.get_instance().get_weight()
    return JsonResponse({'assembly_weight' : assembly_weight,
                         'bin_weight' : bin_weight})

@require_GET
def fsm_state_api(request):
    state = ROS2NodeManager.get_instance().fsm_state
    return JsonResponse({'fsm_state': state})

@require_GET
def ingredient_images_api(request):
    image_dir = os.path.join(settings.STATIC_ROOT or os.path.join(settings.BASE_DIR, 'main/static'), 'segmentation')
    image_pattern = os.path.join(image_dir, '*_check_image.jpg')
    images = glob.glob(image_pattern)
    image_files = [os.path.basename(img) for img in images]
    # Get ingredients in stock (excluding bread)
    stock_path = '/home/snaak/Documents/recipe/stock.yaml'
    try:
        with open(stock_path) as f:
            stock = yaml.safe_load(f)
        ingredients = [name.lower() for name in stock.get('ingredients', {}).keys() if name.lower() != 'bread']
        # Only show images for ingredients in stock
        filtered_images = [img for img in image_files if any(ing in img.lower() for ing in ingredients)]
    except Exception as e:
        filtered_images = image_files  # fallback: show all
    # Always include bread_top and bread_bottom images if present
    for bread_img in ['bread_top_check_image.jpg', 'bread_bottom_check_image.jpg']:
        if bread_img in image_files and bread_img not in filtered_images:
            filtered_images.append(bread_img)
    return JsonResponse({'ingredient_images': filtered_images})
