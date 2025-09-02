from django.urls import path
from . import views

urlpatterns = [
    path('', views.user_page, name='user_page'),
    path('operator/', views.operator_page, name='operator_page'),
]
