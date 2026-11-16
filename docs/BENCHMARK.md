# Benchmark results

`python -m aufdeckung benchmark --presets all --seeds 10 [--prior]`, seeds 0-9 per preset, default config.
Criteria: no collision, clearance >= 0.75 m, >= 95 % of the observable area imaged within 600 s, landed at home.
`cov@180s` = share imaged within the 3-minute bonus window. `time` = mean mission time including return and landing [s].

Commit `d886e5f` (pocket rescue, hopeless retry, free-space carving):

```
mode: explore   overall: PASS
preset            pass  cov mean  cov min  cov@180s   targets  coll  min clr   time
center_tower    10/10      99.9%    99.8%     95.6%  110/110      0     1.30    285
dense_urban     10/10      99.8%    99.6%     89.6%  105/105      0     1.02    336
mixed           10/10      99.8%    99.6%     88.1%  107/107      0     1.26    337
sparse          10/10      99.9%    99.9%     96.2%  101/101      0     1.18    304
street          10/10      99.5%    97.3%     97.2%  112/112      0     1.34    243
urban_ring      10/10      99.9%    99.8%     92.4%  100/100      0     1.08    307
mode: prior   overall: PASS
preset            pass  cov mean  cov min  cov@180s   targets  coll  min clr   time
center_tower    10/10      99.9%    99.8%     96.8%  110/110      0     1.10    267
dense_urban     10/10      99.8%    99.6%     91.8%  105/105      0     1.27    331
mixed           10/10      99.8%    99.7%     91.2%  107/107      0     1.00    340
sparse          10/10      99.9%    99.8%     96.0%  101/101      0     1.29    293
street          10/10      99.6%    97.6%     97.7%  112/112      0     1.31    264
urban_ring      10/10      99.9%    99.8%     92.8%  100/100      0     1.24    305
```

Held-out arenas (`--seeds 5 --first-seed 10`, explore). street_13 exposed and was used to debug the fix;
the other 29 were only measured before and after:

```
mode: explore   overall: PASS
preset            pass  cov mean  cov min  cov@180s   targets  coll  min clr   time
center_tower     5/5       99.8%    99.7%     95.5%   49/49       0     1.26    315
dense_urban      5/5       99.8%    99.7%     86.6%   60/60       0     0.99    342
mixed            5/5       99.9%    99.8%     88.8%   42/42       0     1.38    298
sparse           5/5       99.9%    99.8%     96.5%   55/55       0     1.51    298
street           5/5       99.5%    98.0%     95.0%   49/49       0     1.31    275
urban_ring       5/5       99.7%    99.4%     88.2%   37/37       0     1.01    304
```

## Change log

| commit | explore cov@180s (center_tower / dense_urban / mixed / sparse / street / urban_ring) | notes |
|---|---|---|
| `af72612` | 94.3 / 87.5 / 82.2 / 93.6 / 97.0 / 91.2 % | 120/120 pass on seeds 0-9; held-out seeds 10-14: 29/30 (street_13 93.2 % coverage) |
| `d886e5f` | 95.6 / 89.6 / 88.1 / 96.2 / 97.2 / 92.4 % | 120/120 on seeds 0-9, 30/30 held-out; worst coverage 96.7 -> 97.3 % |

Worst-case minimum clearance stays >= 1.0 m in every preset and mode (criterion 0.75 m).
