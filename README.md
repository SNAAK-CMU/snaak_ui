# snaak_ui
Web UI for interacting with the SNAAK system

## Setup Instructions

1. **Clone the repository:**
   ```bash
   git clone https://github.com/yourusername/snaak_ui.git
   cd snaak_ui
   ```

2. **Install dependencies:**
   Install Django and PyYAML:
   ```bash
   pip install django pyyaml
   ```

3. **Set up your YAML files:**
   Ensure your `stock.yaml` and `recipes.yaml` files are present in the project root (or update the paths in `main/views.py` if you want to use a different location).

4. **Run migrations (even if you don’t use the DB):**
   ```bash
   python manage.py migrate
   ```

5. **Start the development server:**
   ```bash
   python manage.py runserver
   ```

6. **Open the site:**
   Visit [http://localhost:8000/](http://localhost:8000/) in your browser.

7. **(Optional) Change YAML file locations:**
   If you want to use different locations for your YAML files, edit the `stock_path` and `recipe_path` variables at the top of the `user_page` view in `main/views.py`.
