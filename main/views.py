from django.shortcuts import render
import yaml
import os
from django.conf import settings
from django.http import JsonResponse, HttpResponseBadRequest

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
    if request.method == "POST":
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
            return JsonResponse({"status": "success"})
        except Exception as e:
            return HttpResponseBadRequest(str(e))
    return render(request, 'user_page.html', {
        'ingredients': ingredients,
        'recipe': recipe,
        'bread_default': bread_default,
        'stock_path': stock_path,
        'recipe_path': recipe_path
    })

def operator_page(request):
    return render(request, 'operator_page.html')
