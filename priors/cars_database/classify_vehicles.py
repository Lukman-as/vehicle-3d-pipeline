"""
Vehicle classification script.
Categories: sedan, suv, van, bus, truck, heavy_vehicles
"""

import csv
import os
import re
from pathlib import Path

OUTPUT_DIR = Path(__file__).parent / "cleans_data"
INPUT_DIR = Path(__file__).parent

# ─── Keyword-based rules (applied to MODEL string) ───────────────────────────

HEAVY_KEYWORDS = [
    "excavat", "digger", "bulldoz", "dozer", "grader", "loader", "backhoe",
    "forklift", "crane", "dump truck", "cement mixer", "concrete mixer",
    "scraper", "compactor", "paver",
]

BUS_KEYWORDS = [
    " bus", "motorcoach", " coach",
]

VAN_KEYWORDS = [
    " van", "minivan", "cargo van", "passenger van", "transit connect",
    "town & country", "odyssey", "sienna", "caravan", "routan", "express van",
    "savana", "promaster", "metris",
]

TRUCK_KEYWORDS = [
    "pickup", "p/u", "crew cab", "quad cab", "mega cab", "reg cab", "ext cab",
    "double cab", "super cab", "supercab", "supercrew", "super crew",
    "box 2wd", "box 4wd", "box 4x4", "l/box", "s/box", "m/box",
    "f-150", "f-250", "f-350", "f-450", "f-550", "f150", "f250", "f350",
    "silverado", "sierra", "ram 1500", "ram 2500", "ram 3500",
    "tundra", "tacoma", "frontier", "colorado", "canyon", "ridgeline",
    "titan", "ranger", "maverick", "santa cruz",
    "raptor", "lightning",
    "dakota", "dakota p/u",
]

SUV_KEYWORDS = [
    "suv", "cuv", "crossover", "4runner", "4 dr suv", "4dr suv",
    "pilot", "pathfinder", "highlander", "4runner", "sequoia", "land cruiser",
    "expedition", "explorer", "escape", "edge", "bronco",
    "tahoe", "suburban", "traverse", "equinox", "trax", "trailblazer",
    "yukon", "terrain", "acadia", "envoy",
    "escalade", "srx", "xt4", "xt5", "xt6",
    "durango", "journey", "grand cherokee", "wrangler", "cherokee",
    "commander", "compass", "renegade", "gladiator",
    "navigator", "mkx", "mkc", "mkz",
    "murano", "armada", "rogue", "kicks", "xterra", "qashqai",
    "rav4", "4runner", "venza", "fj cruiser", "c-hr", "corolla cross",
    "cx-3", "cx-30", "cx-5", "cx-7", "cx-8", "cx-9", "cx-50", "cx-60", "cx-90",
    "cr-v", "hr-v", "passport", "element", "ridgeline",
    "santa fe", "tucson", "palisade", "nexo", "ioniq 5", "ioniq 7",
    "sportage", "sorento", "telluride", "niro", "soul",
    "forester", "outback", "crosstrek", "ascent", "baja",
    "rx", "nx", "gx", "lx", "ux",
    "rdx", "mdx", "cdx",
    "q2", "q3", "q5", "q7", "q8", "q4 e-tron", "e-tron",
    "x1", "x2", "x3", "x4", "x5", "x6", "x7",
    "gle", "glk", "gla", "glb", "glc", "gls", "g-class", "g class", "g wagon",
    "cayenne", "macan",
    "discovery", "range rover", "defender",
    "flex ", "flex 5",
    "bentayga", "urus", "levante", "stelvio", "e-pace", "f-pace", "i-pace",
    "cullinan", "ghost suv",
    "db11", "dbx",
    "karoq", "kodiaq",
    "police interceptor utility",
    "ecosport",
    "bronco sport", "bronco",
    "atlas", "taos", "tiguan", "touareg",
    "c40 recharge",
    "xc40", "xc60", "xc70", "xc90",
    "301", "3008", "5008",
    "koleos",
    "police utility",
    "4dr cuv", "5dr cuv",
    "allroad",  # Audi A4 Allroad is a wagon/crossover
]

SEDAN_KEYWORDS = [
    "sedan", "4dr", "4 dr", "saloon",
    "liftback",
]

WAGON_TO_SEDAN = [
    # Classified as sedan for our purposes (common wagon/hatch that aren't SUVs)
    "wagon", "hatchback", "hatch", "5dr", "sportback", "gran turismo",
    "3dr", "touring",
]

COUPE_CONV_KEYWORDS = [
    "coupe", "conv", "convertible", "roadster", "cabriolet", "2dr", "spider",
    "spyder", "targa",
]

# ─── Model-level lookup for ambiguous entries ─────────────────────────────────
# (MAKE, partial MODEL) -> category
# All lowercase for matching

MODEL_LOOKUP = {
    # Sports cars / coupes / roadsters -> sedan (catch-all for non-SUV passenger cars)
    ("ferrari", ""): "sedan",
    ("lamborghini", ""): "sedan",
    ("mclaren", ""): "sedan",
    ("pagani", ""): "sedan",
    ("koenigsegg", ""): "sedan",
    ("bugatti", ""): "sedan",
    ("aston martin", ""): "sedan",
    ("bentley", "continental"): "sedan",
    ("bentley", "new continental"): "sedan",
    ("bentley", "mulsanne"): "sedan",
    ("rolls-royce", "ghost"): "sedan",
    ("rolls-royce", "phantom"): "sedan",
    ("rolls-royce", "dawn"): "sedan",
    ("rolls-royce", "wraith"): "sedan",
    ("rolls-royce", "silver"): "sedan",
    ("acura", "nsx"): "sedan",
    ("acura", "integra"): "sedan",
    ("alfa romeo", "4c"): "sedan",
    ("alfa romeo", "giulia"): "sedan",
    ("alfa romeo", "8c"): "sedan",
    ("bmw", "i series i3"): "sedan",
    ("bmw", "i series i8"): "sedan",
    ("bmw", "ix"): "suv",
    ("buick", "cascada"): "sedan",
    ("buick", "regal sportback"): "sedan",
    ("chrysler", "300 srt"): "sedan",
    ("dodge", "dart"): "sedan",
    ("dodge", "viper"): "sedan",
    ("dodge", "challenger"): "sedan",
    ("fiat", "500 abarth"): "sedan",
    ("fiat", "500l"): "sedan",
    ("fiat", "500"): "sedan",
    ("fisker", "karma"): "sedan",
    ("ford", "crown victoria"): "sedan",
    ("ford", "f-150 lightning"): "truck",
    ("ford", "f-150 police"): "truck",
    ("ford", "police interceptor utility"): "suv",
    ("cadillac", "escalade esv"): "suv",
    ("cadillac", "escalade"): "suv",
    ("cadillac", "srx"): "suv",
    ("chevrolet", "silverado"): "truck",
    ("gmc", "sierra"): "truck",
    ("honda", "ridgeline"): "truck",
    ("dodge", "ram"): "truck",
    ("lotus", ""): "sedan",
    ("porsche", "718"): "sedan",
    ("porsche", "911"): "sedan",
    ("porsche", "918"): "sedan",
    ("porsche", "panamera"): "sedan",
    ("porsche", "taycan"): "sedan",
    ("tesla", "model s"): "sedan",
    ("tesla", "model 3"): "sedan",
    ("tesla", "model x"): "suv",
    ("tesla", "model y"): "suv",
    ("tesla", "cybertruck"): "truck",
    ("maserati", "ghibli"): "sedan",
    ("maserati", "granturismo"): "sedan",
    ("maserati", "quattroporte"): "sedan",
    ("maserati", "grancabrio"): "sedan",
    ("bmw", "alpina"): "sedan",
    ("volkswagen", "beetle"): "sedan",
    ("volkswagen", "golf"): "sedan",
    ("volkswagen", "gti"): "sedan",
    ("volkswagen", "gli"): "sedan",
    ("volkswagen", "jetta"): "sedan",
    ("volkswagen", "passat"): "sedan",
    ("volkswagen", "arteon"): "sedan",
    ("volkswagen", "cc"): "sedan",
    ("mini", "clubman"): "sedan",
    ("mini", "cooper"): "sedan",
    ("mini", "paceman"): "sedan",
    ("mini", "coupe"): "sedan",
    ("mini", "roadster"): "sedan",
    ("mini", "hardtop"): "sedan",
    ("mini", "countryman"): "suv",
    ("mini", "crossover"): "suv",
    ("hyundai", "ioniq 5"): "suv",
    ("hyundai", "ioniq 6"): "sedan",
    ("kia", "ev6"): "sedan",
    ("genesis", "g70"): "sedan",
    ("genesis", "g80"): "sedan",
    ("genesis", "g90"): "sedan",
    ("genesis", "gv70"): "suv",
    ("genesis", "gv80"): "suv",
    ("mercedes-benz", "amg gt"): "sedan",
    ("mercedes-benz", "cla"): "sedan",
    ("mercedes-benz", "cls"): "sedan",
    ("ford", "gt"): "sedan",
    ("chevrolet", "corvette"): "sedan",
    ("chevrolet", "camaro"): "sedan",
    ("dodge", "charger"): "sedan",
    ("ford", "mustang"): "sedan",
    ("audi", "r8"): "sedan",
    ("audi", "tt"): "sedan",
    ("audi", "a3"): "sedan",
    ("audi", "a4"): "sedan",
    ("audi", "a5"): "sedan",
    ("audi", "a6"): "sedan",
    ("audi", "a7"): "sedan",
    ("audi", "a8"): "sedan",
    ("audi", "s4"): "sedan",
    ("audi", "s5"): "sedan",
    ("audi", "s6"): "sedan",
    ("audi", "s7"): "sedan",
    ("audi", "s8"): "sedan",
    ("audi", "rs"): "sedan",
    ("bmw", "gran turismo"): "sedan",
    ("bmw", "3 series"): "sedan",
    ("bmw", "5 series"): "sedan",
    ("bmw", "7 series"): "sedan",
    ("bmw", "m series"): "sedan",
    ("bmw", "2 series"): "sedan",
    ("bmw", "4 series"): "sedan",
    ("bmw", "6 series"): "sedan",
    ("bmw", "8 series"): "sedan",
    ("bmw", "1 series"): "sedan",
    ("chevrolet", "malibu"): "sedan",
    ("chevrolet", "impala"): "sedan",
    ("chevrolet", "sonic"): "sedan",
    ("chevrolet", "cruze"): "sedan",
    ("chevrolet", "spark"): "sedan",
    ("chevrolet", "volt"): "sedan",
    ("chevrolet", "bolt"): "sedan",
    ("ford", "fusion"): "sedan",
    ("ford", "fiesta"): "sedan",
    ("ford", "focus"): "sedan",
    ("ford", "taurus"): "sedan",
    ("ford", "c-max"): "sedan",
    ("hyundai", "accent"): "sedan",
    ("hyundai", "elantra"): "sedan",
    ("hyundai", "sonata"): "sedan",
    ("hyundai", "veloster"): "sedan",
    ("hyundai", "azera"): "sedan",
    ("hyundai", "genesis"): "sedan",
    ("hyundai", "equus"): "sedan",
    ("kia", "forte"): "sedan",
    ("kia", "optima"): "sedan",
    ("kia", "stinger"): "sedan",
    ("kia", "k5"): "sedan",
    ("kia", "rio"): "sedan",
    ("mazda", "mazda3"): "sedan",
    ("mazda", "mazda6"): "sedan",
    ("mazda", "mazda2"): "sedan",
    ("honda", "accord"): "sedan",
    ("honda", "civic"): "sedan",
    ("honda", "fit"): "sedan",
    ("honda", "insight"): "sedan",
    ("honda", "clarity"): "sedan",
    ("honda", "accord"): "sedan",
    ("nissan", "altima"): "sedan",
    ("nissan", "maxima"): "sedan",
    ("nissan", "versa"): "sedan",
    ("nissan", "sentra"): "sedan",
    ("nissan", "leaf"): "sedan",
    ("nissan", "gt-r"): "sedan",
    ("nissan", "370z"): "sedan",
    ("nissan", "350z"): "sedan",
    ("subaru", "impreza"): "sedan",
    ("subaru", "legacy"): "sedan",
    ("subaru", "wrx"): "sedan",
    ("subaru", "brz"): "sedan",
    ("toyota", "camry"): "sedan",
    ("toyota", "corolla"): "sedan",
    ("toyota", "avalon"): "sedan",
    ("toyota", "prius"): "sedan",
    ("toyota", "yaris"): "sedan",
    ("toyota", "86"): "sedan",
    ("scion", ""): "sedan",
    ("smart", ""): "sedan",
    ("mitsubishi", "lancer"): "sedan",
    ("mitsubishi", "galant"): "sedan",
    ("mitsubishi", "mirage"): "sedan",
    ("mitsubishi", "eclipse cross"): "suv",
    ("mitsubishi", "outlander sport"): "suv",
    ("mitsubishi", "outlander"): "suv",
    ("mitsubishi", "rvr"): "suv",
    ("lincoln", "mkz"): "sedan",
    ("lincoln", "mks"): "sedan",
    ("lincoln", "town car"): "sedan",
    ("lincoln", "continental"): "sedan",
    ("buick", "lacrosse"): "sedan",
    ("buick", "verano"): "sedan",
    ("buick", "regal"): "sedan",
    ("buick", "lucerne"): "sedan",
    ("cadillac", "cts"): "sedan",
    ("cadillac", "ats"): "sedan",
    ("cadillac", "ct4"): "sedan",
    ("cadillac", "ct5"): "sedan",
    ("cadillac", "ct6"): "sedan",
    ("cadillac", "dts"): "sedan",
    ("cadillac", "xts"): "sedan",
    ("chrysler", "200"): "sedan",
    ("chrysler", "300"): "sedan",
    ("dodge", "avenger"): "sedan",
    ("dodge", "neon"): "sedan",
    ("pontiac", "g6"): "sedan",
    ("pontiac", "g8"): "sedan",
    ("saturn", "aura"): "sedan",
    ("saab", ""): "sedan",
    ("volvo", "s40"): "sedan",
    ("volvo", "s60"): "sedan",
    ("volvo", "s80"): "sedan",
    ("volvo", "s90"): "sedan",
    ("volvo", "v40"): "sedan",
    ("volvo", "v60"): "sedan",
    ("volvo", "v90"): "sedan",
    ("alfa romeo", "159"): "sedan",
    ("peugeot", ""): "sedan",
    ("renault", ""): "sedan",
    ("ram", "promaster city"): "van",
    ("ram", "promaster"): "van",
    ("ram", "cv"): "van",
    ("ram", "ram cv"): "van",
}


def normalize(s: str) -> str:
    return s.lower().strip()


def classify_by_keywords(make: str, model: str):
    m = normalize(model)
    mk = normalize(make)

    for kw in HEAVY_KEYWORDS:
        if kw in m:
            return "heavy_vehicles"

    for kw in BUS_KEYWORDS:
        if kw in m:
            return "bus"

    # Van check (includes minivan)
    for kw in VAN_KEYWORDS:
        if kw in m:
            return "van"

    # Minivan models by name
    if any(x in m for x in ["odyssey", "sienna", "town & country", "caravan",
                              "routan", "quest", "sedona", "carnival"]):
        return "van"

    # Truck
    for kw in TRUCK_KEYWORDS:
        if kw in m:
            return "truck"

    # F-series trucks
    if re.search(r'\bf-\d{3}\b', m) and ("cab" in m or "box" in m or "raptor" in m or "pickup" in m):
        return "truck"

    # SUV
    for kw in SUV_KEYWORDS:
        if kw in m:
            return "suv"

    # Sedan / wagon / hatch / coupe — all under "sedan" umbrella
    for kw in SEDAN_KEYWORDS + WAGON_TO_SEDAN + COUPE_CONV_KEYWORDS:
        if kw in m:
            return "sedan"

    return None


def classify_by_lookup(make: str, model: str):
    mk = normalize(make)
    m = normalize(model)

    # Try progressively shorter model prefixes
    # First exact make+model prefix matches
    best = None
    best_len = -1
    for (lmk, lmod), cat in MODEL_LOOKUP.items():
        if mk == lmk or mk.startswith(lmk):
            if lmod == "" or lmod in m:
                if len(lmod) > best_len:
                    best_len = len(lmod)
                    best = cat

    return best


def classify(make: str, model: str) -> str:
    # 1. Keyword scan
    cat = classify_by_keywords(make, model)
    if cat:
        return cat

    # 2. Lookup table
    cat = classify_by_lookup(make, model)
    if cat:
        return cat

    # 3. Default fallback — treat as sedan (passenger car)
    return "sedan"


def process_files():
    OUTPUT_DIR.mkdir(exist_ok=True)

    categories = ["sedan", "suv", "van", "bus", "truck", "heavy_vehicles"]
    writers = {}
    files = {}

    for cat in categories:
        f = open(OUTPUT_DIR / f"{cat}.csv", "w", newline="", encoding="utf-8")
        files[cat] = f

    csv_files = sorted(INPUT_DIR.glob("*_en.csv"))
    header = None
    rows_by_cat = {c: [] for c in categories}

    for csv_path in csv_files:
        year = csv_path.stem.split("_")[0]
        with open(csv_path, newline="", encoding="latin-1") as f:
            reader = csv.DictReader(f)
            if header is None:
                header = reader.fieldnames + ["YEAR", "CATEGORY"]
            for row in reader:
                make = row.get("MAKE", "").strip()
                model = row.get("MODEL", "").strip()
                if not make or not model:
                    continue
                cat = classify(make, model)
                row["YEAR"] = year
                row["CATEGORY"] = cat
                rows_by_cat[cat].append(row)

    all_fieldnames = header if header else []

    for cat in categories:
        out_path = OUTPUT_DIR / f"{cat}.csv"
        with open(out_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=all_fieldnames)
            writer.writeheader()
            writer.writerows(rows_by_cat[cat])
        print(f"{cat:20s}: {len(rows_by_cat[cat]):5d} rows -> {out_path.name}")

    total = sum(len(v) for v in rows_by_cat.values())
    print(f"\nTotal rows classified: {total}")


if __name__ == "__main__":
    process_files()
