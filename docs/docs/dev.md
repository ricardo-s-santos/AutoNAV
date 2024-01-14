# For developers

If you wish to add more features to AutoNAV, feel free to install it in developer mode as follows.

## Installing for development

With the commands below, one will install all the necessary development dependencies.

```python
git clone https://github.com/Ricardo-Santos-21904332/AutoNAV.git
cd autonav
python -m venv env
source env/bin/activate
pip install -e .[devel]
pre-commit install
```

## Tests

The AutoNAV package contains several tests to assess its quality. These tests can be runned with the command:

```python
pytest
```

A coverage report can also be created with the command:

```python
pytest --cov=autonav --cov-report=html
```
