# This file shouldn't be needed. 
# I had issues installing the package as a user while maintaining editability..
# https://github.com/pypa/pip/issues/7953#issuecomment-860072281
import site
import sys
site.ENABLE_USER_SITE = "--user" in sys.argv[1:]

from setuptools import setup

if __name__ == "__main__":
    setup()