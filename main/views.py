from django.shortcuts import render
import yaml
import os
import math
from django.conf import settings
from django.http import JsonResponse, HttpResponseBadRequest
import threading
from yasmin_msgs.msg import StateMachine
import rclpy
from std_msgs.msg import Bool, String
from django.views.decorators.http import require_GET, require_POST
from snaak_weight_read.srv import ReadWeight
import glob
import json
from django.views.decorators.csrf import csrf_exempt

class ROS2NodeManager:
    _instance = None
    _lock = threading.Lock()

    def __init__(self):
        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node('snaak_ui')
        self.publisher_start_recipe = self.node.create_publisher(Bool, 'snaak_ui/start_recipe', 10)
        self.publisher_toggle_restock = self.node.create_publisher(String, 'snaak_ui/toggle_restock', 10)
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
        # Two bin scales: right for bins 1-3, left for bins 4-6
        self.node.weight_bins_right_client = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_bins_right/read_weight" # TODO: convert this back when state machine can handle two scales
        )
       
        self.node.weight_bins_left_client = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_bins_left/read_weight"
        )
        self.assembly_weight = None
        self.left_bin_weight = None
        self.right_bin_weight = None
        self._weight_lock = threading.Lock()

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
        self.publisher_start_recipe.publish(msg)
    
    def publish_toggle_restock(self, state):
        msg = String()
        msg.data = state
        self.publisher_toggle_restock.publish(msg)

    def get_weight(self, timeout=2.0):
        # Use threading.Event to block until callback sets result
        # have to to async, since node is being spun in seperate thread
        assembly_event = threading.Event()
        right_event = threading.Event()
        left_event = threading.Event()
        result_holder = {'assembly': None, 'left': None, 'right': None}

        def assembly_cb(future):
            result = future.result()
            with self._weight_lock:
                self.assembly_weight = result.weight.data if result else None
                result_holder['assembly'] = self.assembly_weight
            assembly_event.set()

        def right_cb(future):
            result = future.result()
            with self._weight_lock:
                self.right_bin_weight = result.weight.data if result else None
                result_holder['right'] = self.right_bin_weight
            right_event.set()

        def left_cb(future):
            result = future.result()
            with self._weight_lock:
                self.left_bin_weight = result.weight.data if result else None
                result_holder['left'] = self.left_bin_weight
            left_event.set()

        read_weight = ReadWeight.Request()
        future_assembly = self.node.weight_assembly_client.call_async(read_weight)
        future_assembly.add_done_callback(assembly_cb)

        future_right = self.node.weight_bins_right_client.call_async(read_weight)
        future_right.add_done_callback(right_cb)

        future_left = self.node.weight_bins_left_client.call_async(read_weight)
        future_left.add_done_callback(left_cb)

        assembly_event.wait(timeout)
        right_event.wait(timeout)
        left_event.wait(timeout)
        return result_holder['assembly'], result_holder['left'], result_holder['right']
    
def user_page(request):
    # Set your YAML file paths here
    stock_path = '/home/snaak/Documents/recipe/stock.yaml'
    recipe_path = '/home/snaak/Documents/recipe/recipe.yaml'
    restock_warning = None
    try:
        with open(stock_path) as f:
            stock = yaml.safe_load(f)
    except Exception as e:
        return render(request, 'user_page.html', {'error': f'Could not load stock file: {e}'})
    # Find the ingredient with type 'bread'
    bread_slices_raw = 0
    for name, info in stock.get('ingredients', {}).items():
        if info.get('type', '').lower() == 'bread':
            bread_slices_raw = info.get('slices', 0)
            break
    try:
        bread_slices = int(float(bread_slices_raw))
    except (ValueError, TypeError):
        bread_slices = 0
    # Only show warning if in Recipe state and less than 2 slices
    fsm_state = ROS2NodeManager.get_instance().fsm_state
    if bread_slices < 2 and fsm_state == 'Recipe':
        restock_warning = 'Restock is necessary. Please contact an operator.'
    else:
        restock_warning = None
        
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
    bread_ingredients = []
    # Load ingredient info to get weight_per_serving for shredded types
    info_path = '/home/snaak/Documents/recipe/ingredients_info.yaml'
    ingredient_info = {}
    try:
        with open(info_path) as f:
            info_yaml = yaml.safe_load(f)
            if isinstance(info_yaml, dict) and 'ingredients' in info_yaml:
                ingredient_info = info_yaml['ingredients']
    except Exception:
        ingredient_info = {}

    for name, info in stock.get('ingredients', {}).items():
        ing_type = info.get('type', '')
        if ing_type.lower() == 'bread':
            # ensure slices is an int (may be stored as string)
            try:
                bread_slices = int(float(info.get('slices', 0)))
            except Exception:
                bread_slices = 0
            key = name.strip().lower().replace(' ', '_')
            info_meta = ingredient_info.get(key, {}) if isinstance(ingredient_info, dict) else {}
            calories = info_meta.get('calories', 0)
            try:
                calories = float(calories)
            except Exception:
                calories = 0.0
            bread_ingredients.append({'name': name, 'available': bread_slices, 'type': 'bread', 'calories': calories})
        elif ing_type.lower() == 'shredded':
            # For shredded ingredients, stock keeps a 'weight' (grams)
            # coerce stored weight to float (could be string)
            try:
                stock_weight = float(info.get('weight', 0))
            except Exception:
                stock_weight = 0.0
            # Find weight per serving from ingredient_info (fallback to 1g)
            key = name.strip().lower().replace(' ', '_')
            info_meta = ingredient_info.get(key, {}) if isinstance(ingredient_info, dict) else {}
            weight_per_serving = info_meta.get('weight_per_serving') or info_meta.get('weight_per_slice') or 1
            try:
                weight_per_serving = float(weight_per_serving)
            except Exception:
                weight_per_serving = 1.0
            # explicitly floor the division so we always round servings down
            if weight_per_serving > 0:
                available_servings = int(math.floor(stock_weight / weight_per_serving))
                if available_servings < 0:
                    available_servings = 0
            else:
                available_servings = 0
            calories = info_meta.get('calories', 0)
            try:
                calories = float(calories)
            except Exception:
                calories = 0.0
            ingredients.append({'name': name, 'available': available_servings, 'type': 'shredded', 'stock_weight': stock_weight, 'weight_per_serving': weight_per_serving, 'calories': calories})
        else:
            # ensure non-shredded slices count is numeric
            try:
                slices_val = int(float(info.get('slices', 0)))
            except Exception:
                slices_val = 0
            key = name.strip().lower().replace(' ', '_')
            info_meta = ingredient_info.get(key, {}) if isinstance(ingredient_info, dict) else {}
            calories = info_meta.get('calories', 0)
            try:
                calories = float(calories)
            except Exception:
                calories = 0.0
            ingredients.append({'name': name, 'available': slices_val, 'type': ing_type, 'calories': calories})
    bread_default = 2

    # post signifies that form/button submitted on page
    if request.method == "POST": # only one button so do not have to differentiate actions
        try:
            data = yaml.safe_load(request.body) if request.body else request.POST
            # Enforce stock limits and bread default
            new_recipe = []
            # Get selected bread type from POST data
            selected_bread = None
            for bread in bread_ingredients:
                if bread['name'] in data:
                    selected_bread = bread['name']
                    break
            if selected_bread:
                new_recipe.append({selected_bread: 2})
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
    return render(request, 'user_page.html', {
        'ingredients': ingredients,
        'bread_ingredients': bread_ingredients,
        'recipe': recipe,
        'bread_default': bread_default,
        'stock_path': stock_path,
        'recipe_path': recipe_path,
        'restock_warning': restock_warning,
    })

def operator_page(request):
    # Find all *_check_image.jpg files in static/segmentation
    image_dir = os.path.join(settings.STATIC_ROOT or os.path.join(settings.BASE_DIR, 'main/static'), 'segmentation')
    image_pattern = os.path.join(image_dir, '*_check_image.jpg')
    images = glob.glob(image_pattern)
    image_files = [os.path.basename(img) for img in images]
    return render(request, 'operator_page.html', {'ingredient_images': image_files})

@require_GET
def stock_api(request):
    stock_path = '/home/snaak/Documents/recipe/stock.yaml'
    try:
        with open(stock_path) as f:
            stock = yaml.safe_load(f)
        # Extract ingredient stock
        ingredients = stock.get('ingredients', {}) if isinstance(stock, dict) else {}
        # Format: {name: info_dict} for each ingredient
        stock_data = {name: info for name, info in ingredients.items()} if isinstance(ingredients, dict) else {}
        return JsonResponse({'stock': stock_data})
    except Exception as e:
        # Return empty stock but include error message to help debugging on the client
        return JsonResponse({'stock': {}, 'error': str(e)})

@require_GET
def weight_api(request):
    assembly_weight, left_bin_weight, right_bin_weight = ROS2NodeManager.get_instance().get_weight()
    return JsonResponse({'assembly_weight' : assembly_weight,
                         'left_bin_weight' : left_bin_weight,
                         'right_bin_weight' : right_bin_weight})

@require_GET
def fsm_state_api(request):
    state = ROS2NodeManager.get_instance().fsm_state
    return JsonResponse({'fsm_state': state})

@require_GET
def restock_check_api(request):
    """Dynamically check if restock warning should be shown."""
    stock_path = '/home/snaak/Documents/recipe/stock.yaml'
    try:
        with open(stock_path) as f:
            stock = yaml.safe_load(f)
        # Find the ingredient with type 'bread'
        bread_slices_raw = 0
        found_bread = False
        for name, info in stock.get('ingredients', {}).items():
            if info.get('type', '').lower() == 'bread':
                bread_slices_raw = info.get('slices', 0)
                found_bread = True
                break
        try:
            bread_slices = int(float(bread_slices_raw))
        except (ValueError, TypeError):
            bread_slices = 0
        fsm_state = ROS2NodeManager.get_instance().fsm_state
        # Show warning if bread is missing or less than 2 and in Recipe state
        show_warning = (not found_bread or bread_slices < 2) and fsm_state == 'Recipe'
        return JsonResponse({
            'show_warning': show_warning,
            'message': 'Restock is necessary.' if show_warning else ''
        })
    except Exception as e:
        return JsonResponse({'error': str(e)}, status=500)

@require_GET
def ingredient_images_api(request): # TODO: this is going to change when the ingredient names get changed, going to need some other way to do this
    image_dir = os.path.join(settings.STATIC_ROOT or os.path.join(settings.BASE_DIR, 'main/static'), 'segmentation')
    image_pattern = os.path.join(image_dir, '*_check_image.jpg')
    images = glob.glob(image_pattern)
    image_files = [os.path.basename(img) for img in images]
    # Get ingredients in stock (excluding bread)
    filtered_images = []
    for img in ['ham_check_image.jpg', 'cheese_check_image.jpg', 'bread_top_check_image.jpg', 'bread_bottom_check_image.jpg']:
        if img in image_files and img not in filtered_images:
            filtered_images.append(img)
    return JsonResponse({'ingredient_images': filtered_images})

@require_GET
def ingredient_info_api(request):
    info_path = '/home/snaak/Documents/recipe/ingredients_info.yaml'
    try:
        with open(info_path) as f:
            info = yaml.safe_load(f)
        # Return full ingredient info dict for frontend use
        if isinstance(info, dict) and 'ingredients' in info:
            return JsonResponse({'ingredients': info['ingredients']})
        elif isinstance(info, list):
            return JsonResponse({'ingredients': info})
        else:
            return JsonResponse({'ingredients': {}})
    except Exception as e:
        return JsonResponse({'error': str(e)}, status=500)

@csrf_exempt
def update_stock_api(request):
    if request.method != 'POST':
        return JsonResponse({'error': 'POST required'}, status=405)
    try:
        data = json.loads(request.body.decode('utf-8'))
        if not isinstance(data, list):
            return JsonResponse({'error': 'Invalid data format'}, status=400)
        new_stock = {}
        for entry in data:
            ingredient = entry.get('ingredient', {})
            name = ingredient.get('name')
            if name == 'empty':
                continue
            type_ = ingredient.get('type', '')
            bin = ingredient.get('bin', -1)
            # Shredded ingredients are stored by weight (grams)
            if type_ and type_.lower() == 'shredded':
                weight = ingredient.get('weight', ingredient.get('total_weight', 0))
                weight_per_serving = ingredient.get('weight_per_serving', ingredient.get('weight_per_slice', 1))
                try:
                    weight = float(weight)
                except Exception:
                    weight = 0
                try:
                    weight_per_serving = float(weight_per_serving)
                except Exception:
                    weight_per_serving = 1
                if name:
                    new_stock[name] = {
                        'weight': weight,
                        'type': type_,
                        'weight_per_serving': weight_per_serving,
                        'bin': bin
                    }
            else:
                slices = ingredient.get('slices', ingredient.get('total_slices', 0))
                weight_per_slice = ingredient.get('weight_per_slice', 1)
                try:
                    slices = int(slices)
                except Exception:
                    slices = 0
                try:
                    weight_per_slice = float(weight_per_slice)
                except Exception:
                    weight_per_slice = 1
                if name:
                    new_stock[name] = {
                        'slices': slices,
                        'type': type_,
                        'weight_per_slice': weight_per_slice,
                        'bin': bin
                    }
        with open('/home/snaak/Documents/recipe/stock.yaml', 'w') as f:
            yaml.safe_dump({'ingredients': new_stock}, f)
        return JsonResponse({'success': True})
    except Exception as e:
        return JsonResponse({'error': str(e)}, status=500)

@csrf_exempt
@require_GET
def publish_toggle_restock_api(request):
    state = request.GET.get('state', 'Start')
    ROS2NodeManager.get_instance().publish_toggle_restock(state)
    return JsonResponse({'status': 'published', 'state': state})
