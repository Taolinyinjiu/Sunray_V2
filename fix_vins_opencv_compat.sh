#!/usr/bin/env bash

set -euo pipefail

detect_vins_root() {
  local base="$1"

  if [[ -d "$base/localization/vins-fusion-gpu/loop_fusion" && \
        -d "$base/localization/vins-fusion-gpu/vins_estimator" && \
        -d "$base/localization/vins-fusion-gpu/camera_models" ]]; then
    printf '%s\n' "$base/localization/vins-fusion-gpu"
    return 0
  fi

  if [[ -d "$base/loop_fusion" && -d "$base/vins_estimator" && -d "$base/camera_models" ]]; then
    printf '%s\n' "$base"
    return 0
  fi

  return 1
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
vins_root=""

if vins_root="$(detect_vins_root "$PWD")"; then
  :
elif vins_root="$(detect_vins_root "$script_dir")"; then
  :
else
  echo "Cannot locate vins-fusion-gpu."
  echo "Run this script from the repository root or from localization/vins-fusion-gpu."
  exit 1
fi

if ! command -v find >/dev/null 2>&1; then
  echo "find is required but was not found."
  exit 1
fi

if ! command -v grep >/dev/null 2>&1; then
  echo "grep is required but was not found."
  exit 1
fi

if ! command -v perl >/dev/null 2>&1; then
  echo "perl is required but was not found."
  exit 1
fi

pattern='CV_RGB2GRAY|CV_GRAY2RGB|CV_FONT_HERSHEY_SIMPLEX|CV_AA|CV_LOAD_IMAGE_GRAYSCALE|CV_GRAY2BGR|CV_BGR2GRAY|CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_NORMALIZE_IMAGE|CV_CALIB_CB_FILTER_QUADS|CV_CALIB_CB_FAST_CHECK|CV_ADAPTIVE_THRESH_MEAN_C|CV_THRESH_BINARY_INV|CV_THRESH_BINARY|CV_SHAPE_CROSS|CV_SHAPE_RECT|CV_TERMCRIT_EPS|CV_TERMCRIT_ITER|CV_RETR_CCOMP|CV_CHAIN_APPROX_SIMPLE|image\.depth\(\)\s*==\s*3'

candidate_files=()
while IFS= read -r -d '' file; do
  candidate_files+=("$file")
done < <(
  find "$vins_root" \
    \( -path "$vins_root/build" -o -path "$vins_root/devel" \) -prune -o \
    -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.c' -o -name '*.cc' -o -name '*.cpp' \) \
    -print0
)

matched_files=()
if [[ ${#candidate_files[@]} -gt 0 ]]; then
  while IFS= read -r file; do
    matched_files+=("$file")
  done < <(grep -IlE "$pattern" "${candidate_files[@]}" || true)
fi

if [[ ${#matched_files[@]} -eq 0 ]]; then
  echo "No matching legacy OpenCV API usage found under: $vins_root"
  exit 0
fi

echo "Patching legacy OpenCV API usage under: $vins_root"

perl -0pi -e '
BEGIN {
  %replace = (
    "CV_FONT_HERSHEY_SIMPLEX"   => "cv::FONT_HERSHEY_SIMPLEX",
    "CV_LOAD_IMAGE_GRAYSCALE"   => "cv::IMREAD_GRAYSCALE",
    "CV_CALIB_CB_ADAPTIVE_THRESH" => "cv::CALIB_CB_ADAPTIVE_THRESH",
    "CV_CALIB_CB_NORMALIZE_IMAGE" => "cv::CALIB_CB_NORMALIZE_IMAGE",
    "CV_CALIB_CB_FILTER_QUADS"    => "cv::CALIB_CB_FILTER_QUADS",
    "CV_CALIB_CB_FAST_CHECK"      => "cv::CALIB_CB_FAST_CHECK",
    "CV_ADAPTIVE_THRESH_MEAN_C" => "cv::ADAPTIVE_THRESH_MEAN_C",
    "CV_CHAIN_APPROX_SIMPLE"    => "cv::CHAIN_APPROX_SIMPLE",
    "CV_THRESH_BINARY_INV"      => "cv::THRESH_BINARY_INV",
    "CV_THRESH_BINARY"          => "cv::THRESH_BINARY",
    "CV_TERMCRIT_ITER"          => "cv::TermCriteria::MAX_ITER",
    "CV_TERMCRIT_EPS"           => "cv::TermCriteria::EPS",
    "CV_RETR_CCOMP"             => "cv::RETR_CCOMP",
    "CV_GRAY2RGB"               => "cv::COLOR_GRAY2RGB",
    "CV_GRAY2BGR"               => "cv::COLOR_GRAY2BGR",
    "CV_BGR2GRAY"               => "cv::COLOR_BGR2GRAY",
    "CV_RGB2GRAY"               => "cv::COLOR_RGB2GRAY",
    "CV_SHAPE_CROSS"            => "cv::MORPH_CROSS",
    "CV_SHAPE_RECT"             => "cv::MORPH_RECT",
    "CV_AA"                     => "cv::LINE_AA",
  );
}

for my $key (sort { length($b) <=> length($a) } keys %replace) {
  s/\b\Q$key\E\b/$replace{$key}/g;
}

s/\bimage\.depth\(\)\s*==\s*3\b/image.channels() == 3/g;
' "${matched_files[@]}"

echo "Patched ${#matched_files[@]} file(s):"
printf '  %s\n' "${matched_files[@]}"

echo "Done. Rebuild VINS after reviewing the diff."
