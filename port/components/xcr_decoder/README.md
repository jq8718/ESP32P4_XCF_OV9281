# xcr_decoder

ESP32-P4 first-release decoder component. The device source set is intentionally
limited to Code128 and UPC/EAN plus their portable signal-processing helpers.
Codablock F is compiled only as a structural cross-symbology veto so a stacked
symbol cannot be emitted as a truncated Code128 payload; Codablock results are
never exposed by the public API.

Excluded from the component:

- CLI and BMP file loading
- POSIX `SIGALRM`, `setitimer`, `sigsetjmp` and `siglongjmp`
- USPS4ST and its `unsigned __int128` implementation
- MLE research prototypes
- QR, Aztec, DataMatrix, PDF417 and other non-first-release symbologies

The public API is declared in `include/xcr_decoder.h`. Input is a Gray8 buffer
with explicit stride and optional ROI. The decoder returns an `XcrStatus` that
distinguishes no-result, timeout, cancellation, OOM and internal errors.

`reject_ambiguous_retail_numeric` defaults to enabled. It rejects all-numeric
Code128 candidates of EAN-8/UPC-A/EAN-13 length when the retail Mod-10 digit is
invalid. Disable it only when the product must accept arbitrary 8-, 12- or
13-digit Code128 payloads and has another cross-symbology confirmation layer.

Host regression:

```sh
make -f Makefile.host clean all
./build_host/xcr_host_test \
  ../../p0a_audit/artifacts/first_release_coverage.manifest.tsv \
  ../../p0a_audit/xcr_work
```
