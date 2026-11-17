import os, time
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import h2_opt
t = time.time(); fall, hops, rows, ctl = h2_opt.simulate({}, {}); print('first (incl. load)', round(time.time() - t, 2))
t = time.time(); fall, hops, rows, ctl = h2_opt.simulate({'E': 0.1}, {}); print('second', round(time.time() - t, 2), 'sim s', rows[-1]['t'], 'rows', len(rows))
import cProfile, pstats
cProfile.run("h2_opt.simulate({'E': 0.1}, {})", 'prof.out'); pstats.Stats('prof.out').sort_stats('cumtime').print_stats(12); os.remove('prof.out')
