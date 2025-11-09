from django import forms

class IngredientForm(forms.Form):
    def __init__(self, ingredients, *args, **kwargs):
        super().__init__(*args, **kwargs)
        for ingredient in ingredients:
            self.fields[ingredient['name']] = forms.IntegerField(
                min_value=0,
                max_value=ingredient['available'],
                initial=0,
                label=f"{ingredient['name']} (Available: {ingredient['available']})"
            )
