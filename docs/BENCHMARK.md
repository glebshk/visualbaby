# Benchmark results

`python -m aufdeckung benchmark --presets all --seeds 10 [--prior]`, seeds 0-9 per preset, default config.
Criteria: no collision, clearance >= 0.75 m, >= 95 % of the observable area imaged within 600 s, landed at home.
`cov@180s` = share imaged within the 3-minute bonus window. `time` = mean mission time including return and landing [s].

```
mode: explore   overall: PASS
preset            pass  cov mean  cov min  cov@180s   targets  coll  min clr   time
center_tower    10/10      99.8%    99.5%     94.3%  110/110      0     1.02    317
dense_urban     10/10      99.1%    98.0%     87.5%  105/105      0     1.09    335
mixed           10/10      99.1%    97.7%     82.2%  107/107      0     1.04    359
sparse          10/10      99.8%    99.2%     93.6%  101/101      0     1.14    295
street          10/10      99.2%    96.7%     97.0%  112/112      0     1.37    240
urban_ring      10/10      99.6%    99.1%     91.2%  100/100      0     1.10    323
mode: prior   overall: PASS
preset            pass  cov mean  cov min  cov@180s   targets  coll  min clr   time
center_tower    10/10      99.7%    99.0%     94.7%  110/110      0     1.09    299
dense_urban     10/10      99.2%    98.1%     90.5%  105/105      0     1.11    317
mixed           10/10      99.2%    97.8%     86.8%  107/107      0     1.06    354
sparse          10/10      99.8%    99.2%     93.6%  101/101      0     1.01    311
street          10/10      99.3%    96.7%     97.5%  112/112      0     1.25    243
urban_ring      10/10      99.5%    98.9%     92.4%  100/100      0     1.14    318
```
