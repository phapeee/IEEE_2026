#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot normal distributions for selected CSV columns.")
    parser.add_argument(
        "--csv",
        required=True,
        type=Path,
        help="Path to the CSV file exported by the covariance estimator.")
    parser.add_argument(
        "--headers",
        nargs="+",
        required=True,
        help="List of column headers to visualize (one distribution per header).")
    parser.add_argument(
        "--bins",
        type=int,
        default=50,
        help="Number of histogram bins (default: 50).")
    return parser.parse_args()


def load_columns(csv_path: Path, headers: List[str]) -> List[Dict[str, float]]:
    with csv_path.open("r", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        missing = [h for h in headers if h not in reader.fieldnames]
        if missing:
            raise ValueError(f"Missing headers in CSV: {missing}")
        rows = [row for row in reader]
        if not rows:
            raise ValueError("CSV file does not contain any data rows.")
    return rows


def to_float(value: str) -> float:
    try:
        return float(value)
    except ValueError as exc:
        raise ValueError(f"Unable to convert '{value}' to float") from exc


def plot_distribution(values: np.ndarray, label: str, bins: int) -> None:
    fig, ax = plt.subplots()
    ax.hist(values, bins=bins, density=True, alpha=0.6, color="tab:blue", edgecolor="black")

    count = len(values)
    mean = float(np.mean(values))
    variance = float(np.var(values, ddof=1)) if count > 1 else 0.0
    std_dev = float(np.sqrt(variance))

    if std_dev > 0.0:
        x_vals = np.linspace(values.min(), values.max(), 200)
        normal_pdf = (1.0 / (std_dev * np.sqrt(2.0 * np.pi))) * np.exp(
            -0.5 * ((x_vals - mean) / std_dev) ** 2)
        ax.plot(x_vals, normal_pdf, "r-", linewidth=2, label="Normal PDF")
        ax.legend()

    stats_text = (
        f"n = {count}\n"
        f"mean = {mean:.4f}\n"
        f"std = {std_dev:.4f}\n"
        f"var = {variance:.4f}"
    )
    ax.text(
        0.95,
        0.95,
        stats_text,
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="top",
        horizontalalignment="right",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
    )

    ax.set_title(f"{label} distribution")
    ax.set_xlabel(label)
    ax.set_ylabel("Probability Density")
    ax.grid(True, alpha=0.3)


def main() -> None:
    args = parse_args()
    rows = load_columns(args.csv, args.headers)

    for header in args.headers:
        values = np.array([to_float(row[header]) for row in rows], dtype=float)
        if values.size == 0:
            continue
        plot_distribution(values, header, args.bins)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
