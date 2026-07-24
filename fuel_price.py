#!/usr/bin/env python3
"""
Fetch a fuel price from fuelradar.com.au for a given fuel type.
Usage: python3 fuel_price.py <CODE>
  CODE: P98, E10, Diesel (or any partial match on the fuel name/code)

Returns the price in c/L as a float, or exits with code 1 on failure.

Extraction strategy (most durable first):
  1. schema.org JSON-LD (GasStation / makesOffer) - a web standard the site
     emits for SEO, so it survives front-end framework changes.
  2/3. Framework-internal blobs (Next.js / Inertia) as fallbacks only.
Prices are normalised to c/L regardless of how the site encodes them
(dollars 1.667, cents 166.7, or integer tenths 1667).
"""

import sys
import json
import re

URL = "https://fuelradar.com.au/map/station/437aa5eea143da5dd19defc3"
# NOTE: do NOT send an explicit "Accept-Encoding" here. Advertising "br"
# while curl lacks brotli support makes curl fail with rc 61. Passing
# --compressed lets curl advertise only the encodings it can decode.
HEADERS = [
    "-H", "User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36",
    "-H", "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8",
    "-H", "Accept-Language: en-AU,en;q=0.9",
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


def to_cents_per_litre(raw):
    """Normalise a raw price to c/L, whatever unit the site used.

    dollars (1.667) -> *100 ; c/L (166.7) -> as-is ; tenths (1667) -> /10
    """
    v = float(raw)
    if v < 10:          # dollars per litre
        return v * 100
    if v > 500:         # integer tenths of a cent
        return v / 10
    return v            # already c/L


def method_jsonld(html, target):
    """schema.org JSON-LD. Scan every ld+json block for a GasStation."""
    for m in re.finditer(r'<script type="application/ld\+json">(.*?)</script>', html, re.DOTALL):
        try:
            data = json.loads(m.group(1))
        except ValueError:
            continue
        if data.get("@type") != "GasStation":
            continue
        for offer in data.get("makesOffer", []):
            name = offer.get("itemOffered", {}).get("name", "")
            if fuel_matches(name, name, target):
                price = offer.get("price")
                if price is not None:
                    return to_cents_per_litre(price)
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
            return to_cents_per_litre(p.get("Price", 0))
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
            return to_cents_per_litre(p.get("Price", 0))
    return None


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: fuel_price.py <fuel_code>", file=sys.stderr)
        sys.exit(1)

    target = sys.argv[1]
    html = fetch_html()
    if not html:
        print("fetch failed: empty response", file=sys.stderr)
        sys.exit(1)

    for method in (method_jsonld, method_next_data, method_inertia):
        try:
            price = method(html, target)
            if price is not None:
                print(round(float(price), 1))
                sys.exit(0)
        except Exception:
            continue

    print("no price found for %r" % target, file=sys.stderr)
    sys.exit(1)
