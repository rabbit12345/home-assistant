#!/usr/bin/env python3
"""
Fetch a fuel price from fuelradar.com.au for a given fuel type.
Usage: python3 fuel_price.py <CODE>
  CODE: P98, E10, PDSL (or partial match on name, e.g. Diesel)

Returns the price in c/L as a float, or exits with code 1 on failure.
Tries multiple extraction methods in order so it survives site layout changes.
"""

import sys
import json
import re

URL = "https://fuelradar.com.au/map/station/437aa5eea143da5dd19defc3"
HEADERS = [
    "-H", "User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36",
    "-H", "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8",
    "-H", "Accept-Language: en-AU,en;q=0.9",
    "-H", "Accept-Encoding: gzip, deflate, br",
]


def fetch_html():
    import subprocess
    result = subprocess.run(
        ["curl", "-s", "-L", "--compressed"] + HEADERS + [URL],
        capture_output=True, text=True, timeout=30
    )
    return result.stdout


def fuel_matches(name, code, target):
    """True if the fuel name/code matches the requested target."""
    t = target.upper()
    return t in name.upper() or t in code.upper()


def method_jsonld(html, target):
    """Schema.org JSON-LD: <script type="application/ld+json">"""
    m = re.search(r'<script type="application/ld\+json">(.*?)</script>', html, re.DOTALL)
    if not m:
        return None
    data = json.loads(m.group(1))
    for offer in data.get("makesOffer", []):
        name = offer.get("itemOffered", {}).get("name", "")
        if fuel_matches(name, name, target):
            return offer.get("price")
    return None


def method_next_data(html, target):
    """Next.js: <script id="__NEXT_DATA__" type="application/json">"""
    m = re.search(r'<script id="__NEXT_DATA__" type="application/json">(.*?)</script>', html, re.DOTALL)
    if not m:
        return None
    data = json.loads(m.group(1))
    prices = (data.get("props", {})
                  .get("pageProps", {})
                  .get("page", {})
                  .get("props", {})
                  .get("stationData", {})
                  .get("Prices", []))
    for p in prices:
        if fuel_matches(p.get("Name", ""), p.get("Code", ""), target):
            raw = p.get("Price", 0)
            # Price stored as integer (e.g. 1687) or float (e.g. 168.7)
            return raw / 10 if raw > 500 else raw
    return None


def method_inertia(html, target):
    """Inertia.js: data-page attribute on #app"""
    m = re.search(r'data-page="(.*?)"', html)
    if not m:
        return None
    data = json.loads(m.group(1).replace("&quot;", '"'))
    prices = data.get("props", {}).get("stationData", {}).get("Prices", [])
    for p in prices:
        if fuel_matches(p.get("Name", ""), p.get("Code", ""), target):
            raw = p.get("Price", 0)
            return raw / 10 if raw > 500 else raw
    return None


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: fuel_price.py <fuel_code>", file=sys.stderr)
        sys.exit(1)

    target = sys.argv[1]
    html = fetch_html()

    for method in (method_jsonld, method_next_data, method_inertia):
        try:
            price = method(html, target)
            if price is not None:
                print(round(float(price), 1))
                sys.exit(0)
        except Exception:
            continue

    # All methods failed
    sys.exit(1)
