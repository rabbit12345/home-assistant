#!/usr/bin/env python3
"""
Fetch a fuel price for Costco Auburn from the NSW Government FuelCheck API.
Usage:
  python3 fuel_price.py <CODE>              CODE: P98, E10, Diesel
  python3 fuel_price.py --find-station <q>  one-off: list stations matching q

Uses the official FuelCheck API (OAuth2 client-credentials) instead of
scraping a third-party site, so it is not affected by front-end markup
changes or anti-bot protection.

Credentials are read from secrets.yaml (fuelcheck_api_key / fuelcheck_api_secret),
next to this script - never passed on the command line or hardcoded.
"""

import sys
import os
import json
import base64
import uuid
from datetime import datetime
from urllib.request import Request, urlopen
from urllib.error import HTTPError

import yaml

# Costco Auburn's FuelCheck station code. Found once via --find-station and
# hardcoded since it does not change; only prices are fetched per poll.
STATION_CODE = "20550"  # Costco Auburn (Lidcombe), found via --find-station costco

TOKEN_URL = "https://api.onegov.nsw.gov.au/oauth/client_credential/accesstoken?grant_type=client_credentials"
PRICES_URL = "https://api.onegov.nsw.gov.au/FuelPriceCheck/v2/fuel/prices"
STATION_PRICES_URL = "https://api.onegov.nsw.gov.au/FuelPriceCheck/v2/fuel/prices/station/{code}"

SECRETS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "secrets.yaml")

# api.onegov.nsw.gov.au sits behind Cloudflare, which blocks the default
# Python urllib User-Agent (Cloudflare error 1010) regardless of valid auth.
USER_AGENT = "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36"


def load_secrets():
    with open(SECRETS_PATH) as f:
        secrets = yaml.safe_load(f)
    return secrets["fuelcheck_api_key"], secrets["fuelcheck_api_secret"]


def get_token(api_key, api_secret):
    basic = base64.b64encode(f"{api_key}:{api_secret}".encode()).decode()
    req = Request(TOKEN_URL, headers={"Authorization": f"Basic {basic}", "User-Agent": USER_AGENT, "Accept": "application/json"})
    with urlopen(req, timeout=15) as resp:
        return json.load(resp)["access_token"]


def api_headers(api_key, token):
    return {
        "Authorization": f"Bearer {token}",
        "apikey": api_key,
        "Content-Type": "application/json; charset=utf-8",
        "transactionid": str(uuid.uuid4()),
        "requesttimestamp": datetime.now().strftime("%d/%m/%Y %H:%M:%S"),
        "User-Agent": USER_AGENT,
        "Accept": "application/json",
    }


def get_station_prices(api_key, token, station_code):
    url = STATION_PRICES_URL.format(code=station_code)
    req = Request(url, headers=api_headers(api_key, token))
    with urlopen(req, timeout=15) as resp:
        return json.load(resp)


def get_all_prices(api_key, token):
    req = Request(PRICES_URL, headers=api_headers(api_key, token))
    with urlopen(req, timeout=30) as resp:
        return json.load(resp)


# FuelCheck's diesel code is "PDL", not "Diesel".
FUEL_ALIASES = {"DIESEL": "PDL"}


def fuel_matches(code, target):
    target = FUEL_ALIASES.get(target.upper(), target.upper())
    return target == code.upper()


def find_station(query):
    api_key, api_secret = load_secrets()
    token = get_token(api_key, api_secret)
    data = get_all_prices(api_key, token)
    q = query.upper()
    for s in data.get("stations", []):
        name = s.get("name", "")
        address = s.get("address", "")
        if q in name.upper() or q in address.upper():
            print(s.get("code"), "-", name, "-", address)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: fuel_price.py <fuel_code> | --find-station <query>", file=sys.stderr)
        sys.exit(1)

    if sys.argv[1] == "--find-station":
        if len(sys.argv) < 3:
            print("Usage: fuel_price.py --find-station <query>", file=sys.stderr)
            sys.exit(1)
        find_station(sys.argv[2])
        sys.exit(0)

    target = sys.argv[1]
    if not STATION_CODE:
        print("STATION_CODE not set - run: fuel_price.py --find-station costco", file=sys.stderr)
        sys.exit(1)

    try:
        api_key, api_secret = load_secrets()
        token = get_token(api_key, api_secret)
        data = get_station_prices(api_key, token, STATION_CODE)
    except (HTTPError, OSError, KeyError) as e:
        print(f"FuelCheck API error: {e}", file=sys.stderr)
        sys.exit(1)

    for p in data.get("prices", []):
        if fuel_matches(p.get("fueltype", ""), target):
            print(round(float(p["price"]), 1))
            sys.exit(0)

    print("no price found for %r" % target, file=sys.stderr)
    sys.exit(1)
