from django.urls import path
from . import views

urlpatterns = [
    path('', views.user_page, name='user_page'),
    path('operator/', views.operator_page, name='operator_page'),
    path('fsm_state/', views.fsm_state_api, name='fsm_state_api'),
    path('weight_api/', views.weight_api, name='weight_api'),
    path('ingredient_images_api/', views.ingredient_images_api, name='ingredient_images_api'),
    path('stock_api/', views.stock_api, name='stock_api'),
]
