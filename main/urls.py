from django.urls import path
from . import views

urlpatterns = [
    path('', views.user_page, name='user_page'),
    path('operator/', views.operator_page, name='operator_page'),
    path('fsm_state/', views.fsm_state_api, name='fsm_state_api'),
    path('weight_api/', views.weight_api, name='weight_api'),
    path('ingredient_images_api/', views.ingredient_images_api, name='ingredient_images_api'),
    path('stock_api/', views.stock_api, name='stock_api'),
    path('ingredient_info_api/', views.ingredient_info_api, name='ingredient_info_api'),
    path('update_stock_api/', views.update_stock_api, name='update_stock_api'),
    path('publish_toggle_restock_api/', views.publish_toggle_restock_api, name='publish_toggle_restock_api'),
    path('restock_check_api/', views.restock_check_api, name='restock_check_api'),
    path('shredded_log_api/', views.shredded_log_api, name='shredded_log_api'),
]
