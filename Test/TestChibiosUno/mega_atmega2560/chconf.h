<!DOCTYPE html>
<!-- Server: sfn-web-7 -->


    















<!--[if lt IE 7 ]> <html lang="en" class="no-js ie6"> <![endif]-->
<!--[if IE 7 ]>    <html lang="en" class="no-js ie7"> <![endif]-->
<!--[if IE 8 ]>    <html lang="en" class="no-js ie8"> <![endif]-->
<!--[if IE 9 ]>    <html lang="en" class="no-js ie9"> <![endif]-->
<!--[if (gt IE 9)|!(IE)]>-->
<html lang="en" class="no-js"> <!--<![endif]-->
<head>
    <meta content="text/html; charset=UTF-8" http-equiv="content-type"/>
    <title>
  ChibiOS/RT free embedded RTOS / Subversion (main) /
  [r8742]
  /trunk_old/demos/AVR-Arduino-GCC/chconf.h
</title>
    
<meta id="project_name" name="project_name" content='chibios' />
<script src="http://a.fsdn.com/allura/nf/1452191356/_ew_/theme/sftheme/js/sftheme/modernizr.custom.90514.js"></script>
<!--[if lt IE 7 ]>
  <script src="http://a.fsdn.com/allura/nf/1452191356/_ew_/theme/sftheme/js/sftheme/dd_belatedpng.js"></script>
  <script> DD_belatedPNG.fix('img, .png_bg'); //fix any <img> or .png_bg background-images </script>
<![endif]-->
<link href='//fonts.googleapis.com/css?family=Ubuntu:regular' rel='stylesheet' type='text/css'>
    <script type="text/javascript">
        /*jslint onevar: false, nomen: false, evil: true, css: true, plusplus: false, white: false, forin: true, on: true, immed: false */
        /*global confirm, alert, unescape, window, jQuery, $, net, COMSCORE */
    </script>
    
        <!-- ew:head_css -->

    
        <link rel="stylesheet"
                type="text/css"
                href="http://a.fsdn.com/allura/nf/1452191356/_ew_/_slim/css?href=allura%2Fcss%2Fforge%2Fhilite.css%3Ballura%2Fcss%2Fforge%2Ftooltipster.css"
                >
    
        <link rel="stylesheet"
                type="text/css"
                href="http://a.fsdn.com/allura/nf/1452191356/_ew_/allura/css/font-awesome.min.css"
                >
    
        <link rel="stylesheet"
                type="text/css"
                href="http://a.fsdn.com/allura/nf/1452191356/_ew_/theme/sftheme/css/forge.css"
                >
    
        
<!-- /ew:head_css -->

    
    
        <!-- ew:head_js -->

    
        <script type="text/javascript" src="http://a.fsdn.com/allura/nf/1452191356/_ew_/_slim/js?href=allura%2Fjs%2Fjquery-base.js"></script>
    
        
<!-- /ew:head_js -->

    

    
        <style type="text/css">
            #page-body.project---init-- #top_nav { display: none; }

#page-body.project---init-- #nav_menu_holder { display: none; margin-bottom: 0; }

#page-body.project---init-- #content_base {margin-top: 0; }
        </style>
    
    
    <link rel="alternate" type="application/rss+xml" title="RSS" href="/p/chibios/svn/feed.rss"/>
    <link rel="alternate" type="application/atom+xml" title="Atom" href="/p/chibios/svn/feed.atom"/>

    <style>.XCbUNFzfhoHpeWXwtNOrdjIDuQ {
        display: none
    }</style>

    
    
    
    


<script type="text/javascript">
    (function(i,s,o,g,r,a,m){i['GoogleAnalyticsObject']=r;i[r]=i[r]||function(){
            (i[r].q=i[r].q||[]).push(arguments)},i[r].l=1*new Date();a=s.createElement(o),
        m=s.getElementsByTagName(o)[0];a.async=1;a.src=g;m.parentNode.insertBefore(a,m)
    })(window,document,'script','//www.google-analytics.com/analytics.js','ga');

    function _add_tracking(prefix, tracking_id) {
        ga('create', tracking_id, {cookieDomain: 'auto', 'name': prefix});
        
        ga(prefix+'.set', 'dimension9', 'chibios');
        ga(prefix+'.set', 'dimension10', 'svn');
        ga(prefix+'.send', 'pageview');
    }
      _add_tracking('sfnt1', 'UA-32013-6');
      _add_tracking('sfnt2', 'UA-36130941-1');
    
</script>
</head>

<body id="forge">

    
        <!-- ew:body_top_js -->

    
        
<!-- /ew:body_top_js -->

    


<header id="site-header">
    <div class="wrapper">
        <a href="/" class="logo">
            <span>SourceForge</span>
        </a>
        
        <form method="get" action="/directory/">
            <input type="text" id="words" name="q" placeholder="Search">
        </form>
        
        <!--Switch to {language}-->
        <nav id="nav-site">
            <a href="/directory/" title="Browse our software.">Browse</a>
            <a href="/directory/enterprise" title="Browse our Enterprise software.">Enterprise</a>
            <a href="/blog/" title="Read the latest news from the SF HQ.">Blog</a>
            <a href="/jobs?source=header" title="Search 80k+ tech jobs." >Jobs</a>
            <a href="//deals.sourceforge.net/?utm_source=sourceforge&amp;utm_medium=navbar&amp;utm_campaign=homepage" title="Discover and Save on the Best Gear, Gadgets, and Software" class="featured-link" target="_blank">Deals</a>
            <a href="/support" title="Contact us for help and feedback.">Help</a>
        </nav>
        <nav id="nav-account">
            
              <div class="logged_out">
                <a href="/auth/">Log In</a>
                <span>or</span>
                <a href="https://sourceforge.net/user/registration/">Join</a>
              </div>
            
        </nav>
        
    </div>
</header>
<header id="site-sec-header">
    <div class="wrapper">
        <nav id="nav-hubs">
            <h4>Solution Centers</h4>
            <a href="http://goparallel.sourceforge.net/">Go Parallel</a>
        </nav>
        <nav id="nav-collateral">
            <a href="http://library.slashdotmedia.com/?source=sfnet_header">Resources</a>
            
            <a href="">Newsletters</a>
            
        </nav>
    </div>
</header>

    
    
    

<section id="page-body" class=" neighborhood-Projects project-chibios mountpoint-svn">
    <div id="nav_menu_holder">
        
            



    
    
    
    
    <nav id="breadcrumbs">
        <ul>
            <li itemscope itemtype="http://data-vocabulary.org/Breadcrumb"><a itemprop="url" href="/">Home</a></li>
            <li itemscope itemtype="http://data-vocabulary.org/Breadcrumb"><a itemprop="url" href="/directory">Browse</a></li>
            
            
                
            
            
            
                <li itemscope itemtype="http://data-vocabulary.org/Breadcrumb"><a itemprop="url" href="/p/">Projects</a></li>
                
            
            
                <li itemscope itemtype="http://data-vocabulary.org/Breadcrumb"><a itemprop="url" href="/p/chibios/">ChibiOS/RT free embedded RTOS</a></li>
                
            
            
                <li itemscope itemtype="http://data-vocabulary.org/Breadcrumb">Subversion (main)</li>
                
            
        </ul>
    </nav>
    
    
    
  
    
      <img src="/p/chibios/icon?2013-04-06 11:40:13+00:00" class="project_icon" alt="Project Logo">
    
    <h1 class="project_title">
        <a href="/p/chibios/" class="project_link">ChibiOS/RT free embedded RTOS</a>
    </h1>
    
    
    
    <h2 class="project_summary with-icon">
        
    </h2>
    
    <div class="brought-by with-icon">
        Brought to you by:
        
        
            
                <a href="/u/gdisirio/">gdisirio</a>
            </div>
    

        
    </div>
    <div id="top_nav" class="">
        
            
<ul class="dropdown">
  
    <li class="">
        <a href="/projects/chibios/" class="tool-summary">
            Summary
        </a>
        
        
    </li>
	
    <li class="">
        <a href="/projects/chibios/files/" class="tool-files">
            Files
        </a>
        
        
    </li>
	
    <li class="">
        <a href="/projects/chibios/reviews" class="tool-reviews">
            Reviews
        </a>
        
        
    </li>
	
    <li class="">
        <a href="/projects/chibios/support" class="tool-support">
            Support
        </a>
        
        
    </li>
	
    <li class="">
        <a href="/p/chibios/wiki/" class="tool-wiki">
            Wiki
        </a>
        
        
    </li>
	
    <li class="">
        <a href="/p/chibios/_list/tickets" class="tool-tickets">
            Tickets ▾
        </a>
        
        
            <ul>
                
                    <li class=""><a href="/p/chibios/bugs/">Bugs</a></li>
                
                    <li class=""><a href="/p/chibios/feature-requests/">Feature Requests</a></li>
                
            </ul>
        
    </li>
	
    <li class="">
        <a href="/p/chibios/news/" class="tool-blog">
            News
        </a>
        
        
    </li>
	
    <li class="">
        <a href="/p/chibios/discussion/" class="tool-discussion">
            Discussion
        </a>
        
        
    </li>
	
    <li class="selected">
        <a href="/p/chibios/svn/" class="tool-svn">
            Subversion (main)
        </a>
        
        
    </li>
	
    <li class="">
        <a href="/p/chibios/donate/" class="tool-link">
            Donate
        </a>
        
        
    </li>
	
</ul>

        
    </div>
    <div id="content_base">
        
            
                
                    


<div id="sidebar">
  
    <div>&nbsp;</div>
  
    
    
      
      
        
    
      <ul class="sidebarmenu">
      
    
  <li>
      
        <a class="icon" href="/p/chibios/svn/commit_browser" title="Browse Commits"><i class="fa fa-list"></i>
      
      <span>Browse Commits</span>
      </a>
  </li>
  
      
    
    
      </ul>
      
    
    
    
</div>
                
                
            
            
                
            
            <div class="grid-20 pad">
                <h2 class="dark title">
<a href="/p/chibios/svn/8742/">[r8742]</a>:

  
  
    <a href="./../../">trunk_old</a> /
    
  
    <a href="./../">demos</a> /
    
  
    <a href="./">AVR-Arduino-GCC</a> /
    
  
 chconf.h

                    <!-- actions -->
                    <small>
                        

    
    <a class="icon" href="#" id="maximize-content" title="Maximize"><i class="fa fa-expand"></i>&nbsp;Maximize</a>
    <a class="icon" href="#" id="restore-content" title="Restore"><i class="fa fa-compress"></i>&nbsp;Restore</a>
<a class="icon" href="/p/chibios/svn/8742/log/?path=/trunk_old/demos/AVR-Arduino-GCC/chconf.h" title="History"><i class="fa fa-calendar"></i>&nbsp;History</a>

                    </small>
                    <!-- /actions -->
                </h2>
                
                <div>
                    
  

                    
  
    <p><a href="?format=raw">Download this file</a></p>
    <div class="clip grid-19 codebrowser">
      <h3>
        532 lines (475 with data), 17.2 kB
      </h3>
      
        <table class="codehilitetable"><tr><td class="linenos"><div class="linenodiv"><pre>  1
  2
  3
  4
  5
  6
  7
  8
  9
 10
 11
 12
 13
 14
 15
 16
 17
 18
 19
 20
 21
 22
 23
 24
 25
 26
 27
 28
 29
 30
 31
 32
 33
 34
 35
 36
 37
 38
 39
 40
 41
 42
 43
 44
 45
 46
 47
 48
 49
 50
 51
 52
 53
 54
 55
 56
 57
 58
 59
 60
 61
 62
 63
 64
 65
 66
 67
 68
 69
 70
 71
 72
 73
 74
 75
 76
 77
 78
 79
 80
 81
 82
 83
 84
 85
 86
 87
 88
 89
 90
 91
 92
 93
 94
 95
 96
 97
 98
 99
100
101
102
103
104
105
106
107
108
109
110
111
112
113
114
115
116
117
118
119
120
121
122
123
124
125
126
127
128
129
130
131
132
133
134
135
136
137
138
139
140
141
142
143
144
145
146
147
148
149
150
151
152
153
154
155
156
157
158
159
160
161
162
163
164
165
166
167
168
169
170
171
172
173
174
175
176
177
178
179
180
181
182
183
184
185
186
187
188
189
190
191
192
193
194
195
196
197
198
199
200
201
202
203
204
205
206
207
208
209
210
211
212
213
214
215
216
217
218
219
220
221
222
223
224
225
226
227
228
229
230
231
232
233
234
235
236
237
238
239
240
241
242
243
244
245
246
247
248
249
250
251
252
253
254
255
256
257
258
259
260
261
262
263
264
265
266
267
268
269
270
271
272
273
274
275
276
277
278
279
280
281
282
283
284
285
286
287
288
289
290
291
292
293
294
295
296
297
298
299
300
301
302
303
304
305
306
307
308
309
310
311
312
313
314
315
316
317
318
319
320
321
322
323
324
325
326
327
328
329
330
331
332
333
334
335
336
337
338
339
340
341
342
343
344
345
346
347
348
349
350
351
352
353
354
355
356
357
358
359
360
361
362
363
364
365
366
367
368
369
370
371
372
373
374
375
376
377
378
379
380
381
382
383
384
385
386
387
388
389
390
391
392
393
394
395
396
397
398
399
400
401
402
403
404
405
406
407
408
409
410
411
412
413
414
415
416
417
418
419
420
421
422
423
424
425
426
427
428
429
430
431
432
433
434
435
436
437
438
439
440
441
442
443
444
445
446
447
448
449
450
451
452
453
454
455
456
457
458
459
460
461
462
463
464
465
466
467
468
469
470
471
472
473
474
475
476
477
478
479
480
481
482
483
484
485
486
487
488
489
490
491
492
493
494
495
496
497
498
499
500
501
502
503
504
505
506
507
508
509
510
511
512
513
514
515
516
517
518
519
520
521
522
523
524
525
526
527
528
529
530
531</pre></div></td><td class="code"><div class="codehilite"><pre><div id="l1" class="code_block"><span class="cm">/*</span>
</div><div id="l2" class="code_block"><span class="cm">    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio</span>
</div><div id="l3" class="code_block">
</div><div id="l4" class="code_block"><span class="cm">    Licensed under the Apache License, Version 2.0 (the &quot;License&quot;);</span>
</div><div id="l5" class="code_block"><span class="cm">    you may not use this file except in compliance with the License.</span>
</div><div id="l6" class="code_block"><span class="cm">    You may obtain a copy of the License at</span>
</div><div id="l7" class="code_block">
</div><div id="l8" class="code_block"><span class="cm">        http://www.apache.org/licenses/LICENSE-2.0</span>
</div><div id="l9" class="code_block">
</div><div id="l10" class="code_block"><span class="cm">    Unless required by applicable law or agreed to in writing, software</span>
</div><div id="l11" class="code_block"><span class="cm">    distributed under the License is distributed on an &quot;AS IS&quot; BASIS,</span>
</div><div id="l12" class="code_block"><span class="cm">    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.</span>
</div><div id="l13" class="code_block"><span class="cm">    See the License for the specific language governing permissions and</span>
</div><div id="l14" class="code_block"><span class="cm">    limitations under the License.</span>
</div><div id="l15" class="code_block"><span class="cm">*/</span>
</div><div id="l16" class="code_block">
</div><div id="l17" class="code_block"><span class="cm">/**</span>
</div><div id="l18" class="code_block"><span class="cm"> * @file    templates/chconf.h</span>
</div><div id="l19" class="code_block"><span class="cm"> * @brief   Configuration file template.</span>
</div><div id="l20" class="code_block"><span class="cm"> * @details A copy of this file must be placed in each project directory, it</span>
</div><div id="l21" class="code_block"><span class="cm"> *          contains the application specific kernel settings.</span>
</div><div id="l22" class="code_block"><span class="cm"> *</span>
</div><div id="l23" class="code_block"><span class="cm"> * @addtogroup config</span>
</div><div id="l24" class="code_block"><span class="cm"> * @details Kernel related settings and hooks.</span>
</div><div id="l25" class="code_block"><span class="cm"> * @{</span>
</div><div id="l26" class="code_block"><span class="cm"> */</span>
</div><div id="l27" class="code_block">
</div><div id="l28" class="code_block"><span class="cp">#ifndef _CHCONF_H_</span>
</div><div id="l29" class="code_block"><span class="cp">#define _CHCONF_H_</span>
</div><div id="l30" class="code_block">
</div><div id="l31" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l32" class="code_block"><span class="cm">/**</span>
</div><div id="l33" class="code_block"><span class="cm"> * @name Kernel parameters and options</span>
</div><div id="l34" class="code_block"><span class="cm"> * @{</span>
</div><div id="l35" class="code_block"><span class="cm"> */</span>
</div><div id="l36" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l37" class="code_block">
</div><div id="l38" class="code_block"><span class="cm">/**</span>
</div><div id="l39" class="code_block"><span class="cm"> * @brief   System tick frequency.</span>
</div><div id="l40" class="code_block"><span class="cm"> * @details Frequency of the system timer that drives the system ticks. This</span>
</div><div id="l41" class="code_block"><span class="cm"> *          setting also defines the system tick time unit.</span>
</div><div id="l42" class="code_block"><span class="cm"> */</span>
</div><div id="l43" class="code_block"><span class="cp">#if !defined(CH_FREQUENCY) || defined(__DOXYGEN__)</span>
</div><div id="l44" class="code_block"><span class="cp">#define CH_FREQUENCY                    100</span>
</div><div id="l45" class="code_block"><span class="cp">#endif</span>
</div><div id="l46" class="code_block">
</div><div id="l47" class="code_block"><span class="cm">/**</span>
</div><div id="l48" class="code_block"><span class="cm"> * @brief   Round robin interval.</span>
</div><div id="l49" class="code_block"><span class="cm"> * @details This constant is the number of system ticks allowed for the</span>
</div><div id="l50" class="code_block"><span class="cm"> *          threads before preemption occurs. Setting this value to zero</span>
</div><div id="l51" class="code_block"><span class="cm"> *          disables the preemption for threads with equal priority and the</span>
</div><div id="l52" class="code_block"><span class="cm"> *          round robin becomes cooperative. Note that higher priority</span>
</div><div id="l53" class="code_block"><span class="cm"> *          threads can still preempt, the kernel is always preemptive.</span>
</div><div id="l54" class="code_block"><span class="cm"> *</span>
</div><div id="l55" class="code_block"><span class="cm"> * @note    Disabling the round robin preemption makes the kernel more compact</span>
</div><div id="l56" class="code_block"><span class="cm"> *          and generally faster.</span>
</div><div id="l57" class="code_block"><span class="cm"> */</span>
</div><div id="l58" class="code_block"><span class="cp">#if !defined(CH_TIME_QUANTUM) || defined(__DOXYGEN__)</span>
</div><div id="l59" class="code_block"><span class="cp">#define CH_TIME_QUANTUM                 20</span>
</div><div id="l60" class="code_block"><span class="cp">#endif</span>
</div><div id="l61" class="code_block">
</div><div id="l62" class="code_block"><span class="cm">/**</span>
</div><div id="l63" class="code_block"><span class="cm"> * @brief   Managed RAM size.</span>
</div><div id="l64" class="code_block"><span class="cm"> * @details Size of the RAM area to be managed by the OS. If set to zero</span>
</div><div id="l65" class="code_block"><span class="cm"> *          then the whole available RAM is used. The core memory is made</span>
</div><div id="l66" class="code_block"><span class="cm"> *          available to the heap allocator and/or can be used directly through</span>
</div><div id="l67" class="code_block"><span class="cm"> *          the simplified core memory allocator.</span>
</div><div id="l68" class="code_block"><span class="cm"> *</span>
</div><div id="l69" class="code_block"><span class="cm"> * @note    In order to let the OS manage the whole RAM the linker script must</span>
</div><div id="l70" class="code_block"><span class="cm"> *          provide the @p __heap_base__ and @p __heap_end__ symbols.</span>
</div><div id="l71" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_MEMCORE.</span>
</div><div id="l72" class="code_block"><span class="cm"> */</span>
</div><div id="l73" class="code_block"><span class="cp">#if !defined(CH_MEMCORE_SIZE) || defined(__DOXYGEN__)</span>
</div><div id="l74" class="code_block"><span class="cp">#define CH_MEMCORE_SIZE                 128</span>
</div><div id="l75" class="code_block"><span class="cp">#endif</span>
</div><div id="l76" class="code_block">
</div><div id="l77" class="code_block"><span class="cm">/**</span>
</div><div id="l78" class="code_block"><span class="cm"> * @brief   Idle thread automatic spawn suppression.</span>
</div><div id="l79" class="code_block"><span class="cm"> * @details When this option is activated the function @p chSysInit()</span>
</div><div id="l80" class="code_block"><span class="cm"> *          does not spawn the idle thread automatically. The application has</span>
</div><div id="l81" class="code_block"><span class="cm"> *          then the responsibility to do one of the following:</span>
</div><div id="l82" class="code_block"><span class="cm"> *          - Spawn a custom idle thread at priority @p IDLEPRIO.</span>
</div><div id="l83" class="code_block"><span class="cm"> *          - Change the main() thread priority to @p IDLEPRIO then enter</span>
</div><div id="l84" class="code_block"><span class="cm"> *            an endless loop. In this scenario the @p main() thread acts as</span>
</div><div id="l85" class="code_block"><span class="cm"> *            the idle thread.</span>
</div><div id="l86" class="code_block"><span class="cm"> *          .</span>
</div><div id="l87" class="code_block"><span class="cm"> * @note    Unless an idle thread is spawned the @p main() thread must not</span>
</div><div id="l88" class="code_block"><span class="cm"> *          enter a sleep state.</span>
</div><div id="l89" class="code_block"><span class="cm"> */</span>
</div><div id="l90" class="code_block"><span class="cp">#if !defined(CH_NO_IDLE_THREAD) || defined(__DOXYGEN__)</span>
</div><div id="l91" class="code_block"><span class="cp">#define CH_NO_IDLE_THREAD               FALSE</span>
</div><div id="l92" class="code_block"><span class="cp">#endif</span>
</div><div id="l93" class="code_block">
</div><div id="l94" class="code_block"><span class="cm">/** @} */</span>
</div><div id="l95" class="code_block">
</div><div id="l96" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l97" class="code_block"><span class="cm">/**</span>
</div><div id="l98" class="code_block"><span class="cm"> * @name Performance options</span>
</div><div id="l99" class="code_block"><span class="cm"> * @{</span>
</div><div id="l100" class="code_block"><span class="cm"> */</span>
</div><div id="l101" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l102" class="code_block">
</div><div id="l103" class="code_block"><span class="cm">/**</span>
</div><div id="l104" class="code_block"><span class="cm"> * @brief   OS optimization.</span>
</div><div id="l105" class="code_block"><span class="cm"> * @details If enabled then time efficient rather than space efficient code</span>
</div><div id="l106" class="code_block"><span class="cm"> *          is used when two possible implementations exist.</span>
</div><div id="l107" class="code_block"><span class="cm"> *</span>
</div><div id="l108" class="code_block"><span class="cm"> * @note    This is not related to the compiler optimization options.</span>
</div><div id="l109" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l110" class="code_block"><span class="cm"> */</span>
</div><div id="l111" class="code_block"><span class="cp">#if !defined(CH_OPTIMIZE_SPEED) || defined(__DOXYGEN__)</span>
</div><div id="l112" class="code_block"><span class="cp">#define CH_OPTIMIZE_SPEED               TRUE</span>
</div><div id="l113" class="code_block"><span class="cp">#endif</span>
</div><div id="l114" class="code_block">
</div><div id="l115" class="code_block"><span class="cm">/** @} */</span>
</div><div id="l116" class="code_block">
</div><div id="l117" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l118" class="code_block"><span class="cm">/**</span>
</div><div id="l119" class="code_block"><span class="cm"> * @name Subsystem options</span>
</div><div id="l120" class="code_block"><span class="cm"> * @{</span>
</div><div id="l121" class="code_block"><span class="cm"> */</span>
</div><div id="l122" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l123" class="code_block">
</div><div id="l124" class="code_block"><span class="cm">/**</span>
</div><div id="l125" class="code_block"><span class="cm"> * @brief   Threads registry APIs.</span>
</div><div id="l126" class="code_block"><span class="cm"> * @details If enabled then the registry APIs are included in the kernel.</span>
</div><div id="l127" class="code_block"><span class="cm"> *</span>
</div><div id="l128" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l129" class="code_block"><span class="cm"> */</span>
</div><div id="l130" class="code_block"><span class="cp">#if !defined(CH_USE_REGISTRY) || defined(__DOXYGEN__)</span>
</div><div id="l131" class="code_block"><span class="cp">#define CH_USE_REGISTRY                 TRUE</span>
</div><div id="l132" class="code_block"><span class="cp">#endif</span>
</div><div id="l133" class="code_block">
</div><div id="l134" class="code_block"><span class="cm">/**</span>
</div><div id="l135" class="code_block"><span class="cm"> * @brief   Threads synchronization APIs.</span>
</div><div id="l136" class="code_block"><span class="cm"> * @details If enabled then the @p chThdWait() function is included in</span>
</div><div id="l137" class="code_block"><span class="cm"> *          the kernel.</span>
</div><div id="l138" class="code_block"><span class="cm"> *</span>
</div><div id="l139" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l140" class="code_block"><span class="cm"> */</span>
</div><div id="l141" class="code_block"><span class="cp">#if !defined(CH_USE_WAITEXIT) || defined(__DOXYGEN__)</span>
</div><div id="l142" class="code_block"><span class="cp">#define CH_USE_WAITEXIT                 TRUE</span>
</div><div id="l143" class="code_block"><span class="cp">#endif</span>
</div><div id="l144" class="code_block">
</div><div id="l145" class="code_block"><span class="cm">/**</span>
</div><div id="l146" class="code_block"><span class="cm"> * @brief   Semaphores APIs.</span>
</div><div id="l147" class="code_block"><span class="cm"> * @details If enabled then the Semaphores APIs are included in the kernel.</span>
</div><div id="l148" class="code_block"><span class="cm"> *</span>
</div><div id="l149" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l150" class="code_block"><span class="cm"> */</span>
</div><div id="l151" class="code_block"><span class="cp">#if !defined(CH_USE_SEMAPHORES) || defined(__DOXYGEN__)</span>
</div><div id="l152" class="code_block"><span class="cp">#define CH_USE_SEMAPHORES               TRUE</span>
</div><div id="l153" class="code_block"><span class="cp">#endif</span>
</div><div id="l154" class="code_block">
</div><div id="l155" class="code_block"><span class="cm">/**</span>
</div><div id="l156" class="code_block"><span class="cm"> * @brief   Semaphores queuing mode.</span>
</div><div id="l157" class="code_block"><span class="cm"> * @details If enabled then the threads are enqueued on semaphores by</span>
</div><div id="l158" class="code_block"><span class="cm"> *          priority rather than in FIFO order.</span>
</div><div id="l159" class="code_block"><span class="cm"> *</span>
</div><div id="l160" class="code_block"><span class="cm"> * @note    The default is @p FALSE. Enable this if you have special requirements.</span>
</div><div id="l161" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_SEMAPHORES.</span>
</div><div id="l162" class="code_block"><span class="cm"> */</span>
</div><div id="l163" class="code_block"><span class="cp">#if !defined(CH_USE_SEMAPHORES_PRIORITY) || defined(__DOXYGEN__)</span>
</div><div id="l164" class="code_block"><span class="cp">#define CH_USE_SEMAPHORES_PRIORITY      FALSE</span>
</div><div id="l165" class="code_block"><span class="cp">#endif</span>
</div><div id="l166" class="code_block">
</div><div id="l167" class="code_block"><span class="cm">/**</span>
</div><div id="l168" class="code_block"><span class="cm"> * @brief   Atomic semaphore API.</span>
</div><div id="l169" class="code_block"><span class="cm"> * @details If enabled then the semaphores the @p chSemSignalWait() API</span>
</div><div id="l170" class="code_block"><span class="cm"> *          is included in the kernel.</span>
</div><div id="l171" class="code_block"><span class="cm"> *</span>
</div><div id="l172" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l173" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_SEMAPHORES.</span>
</div><div id="l174" class="code_block"><span class="cm"> */</span>
</div><div id="l175" class="code_block"><span class="cp">#if !defined(CH_USE_SEMSW) || defined(__DOXYGEN__)</span>
</div><div id="l176" class="code_block"><span class="cp">#define CH_USE_SEMSW                    TRUE</span>
</div><div id="l177" class="code_block"><span class="cp">#endif</span>
</div><div id="l178" class="code_block">
</div><div id="l179" class="code_block"><span class="cm">/**</span>
</div><div id="l180" class="code_block"><span class="cm"> * @brief   Mutexes APIs.</span>
</div><div id="l181" class="code_block"><span class="cm"> * @details If enabled then the mutexes APIs are included in the kernel.</span>
</div><div id="l182" class="code_block"><span class="cm"> *</span>
</div><div id="l183" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l184" class="code_block"><span class="cm"> */</span>
</div><div id="l185" class="code_block"><span class="cp">#if !defined(CH_USE_MUTEXES) || defined(__DOXYGEN__)</span>
</div><div id="l186" class="code_block"><span class="cp">#define CH_USE_MUTEXES                  TRUE</span>
</div><div id="l187" class="code_block"><span class="cp">#endif</span>
</div><div id="l188" class="code_block">
</div><div id="l189" class="code_block"><span class="cm">/**</span>
</div><div id="l190" class="code_block"><span class="cm"> * @brief   Conditional Variables APIs.</span>
</div><div id="l191" class="code_block"><span class="cm"> * @details If enabled then the conditional variables APIs are included</span>
</div><div id="l192" class="code_block"><span class="cm"> *          in the kernel.</span>
</div><div id="l193" class="code_block"><span class="cm"> *</span>
</div><div id="l194" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l195" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_MUTEXES.</span>
</div><div id="l196" class="code_block"><span class="cm"> */</span>
</div><div id="l197" class="code_block"><span class="cp">#if !defined(CH_USE_CONDVARS) || defined(__DOXYGEN__)</span>
</div><div id="l198" class="code_block"><span class="cp">#define CH_USE_CONDVARS                 TRUE</span>
</div><div id="l199" class="code_block"><span class="cp">#endif</span>
</div><div id="l200" class="code_block">
</div><div id="l201" class="code_block"><span class="cm">/**</span>
</div><div id="l202" class="code_block"><span class="cm"> * @brief   Conditional Variables APIs with timeout.</span>
</div><div id="l203" class="code_block"><span class="cm"> * @details If enabled then the conditional variables APIs with timeout</span>
</div><div id="l204" class="code_block"><span class="cm"> *          specification are included in the kernel.</span>
</div><div id="l205" class="code_block"><span class="cm"> *</span>
</div><div id="l206" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l207" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_CONDVARS.</span>
</div><div id="l208" class="code_block"><span class="cm"> */</span>
</div><div id="l209" class="code_block"><span class="cp">#if !defined(CH_USE_CONDVARS_TIMEOUT) || defined(__DOXYGEN__)</span>
</div><div id="l210" class="code_block"><span class="cp">#define CH_USE_CONDVARS_TIMEOUT         TRUE</span>
</div><div id="l211" class="code_block"><span class="cp">#endif</span>
</div><div id="l212" class="code_block">
</div><div id="l213" class="code_block"><span class="cm">/**</span>
</div><div id="l214" class="code_block"><span class="cm"> * @brief   Events Flags APIs.</span>
</div><div id="l215" class="code_block"><span class="cm"> * @details If enabled then the event flags APIs are included in the kernel.</span>
</div><div id="l216" class="code_block"><span class="cm"> *</span>
</div><div id="l217" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l218" class="code_block"><span class="cm"> */</span>
</div><div id="l219" class="code_block"><span class="cp">#if !defined(CH_USE_EVENTS) || defined(__DOXYGEN__)</span>
</div><div id="l220" class="code_block"><span class="cp">#define CH_USE_EVENTS                   TRUE</span>
</div><div id="l221" class="code_block"><span class="cp">#endif</span>
</div><div id="l222" class="code_block">
</div><div id="l223" class="code_block"><span class="cm">/**</span>
</div><div id="l224" class="code_block"><span class="cm"> * @brief   Events Flags APIs with timeout.</span>
</div><div id="l225" class="code_block"><span class="cm"> * @details If enabled then the events APIs with timeout specification</span>
</div><div id="l226" class="code_block"><span class="cm"> *          are included in the kernel.</span>
</div><div id="l227" class="code_block"><span class="cm"> *</span>
</div><div id="l228" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l229" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_EVENTS.</span>
</div><div id="l230" class="code_block"><span class="cm"> */</span>
</div><div id="l231" class="code_block"><span class="cp">#if !defined(CH_USE_EVENTS_TIMEOUT) || defined(__DOXYGEN__)</span>
</div><div id="l232" class="code_block"><span class="cp">#define CH_USE_EVENTS_TIMEOUT           TRUE</span>
</div><div id="l233" class="code_block"><span class="cp">#endif</span>
</div><div id="l234" class="code_block">
</div><div id="l235" class="code_block"><span class="cm">/**</span>
</div><div id="l236" class="code_block"><span class="cm"> * @brief   Synchronous Messages APIs.</span>
</div><div id="l237" class="code_block"><span class="cm"> * @details If enabled then the synchronous messages APIs are included</span>
</div><div id="l238" class="code_block"><span class="cm"> *          in the kernel.</span>
</div><div id="l239" class="code_block"><span class="cm"> *</span>
</div><div id="l240" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l241" class="code_block"><span class="cm"> */</span>
</div><div id="l242" class="code_block"><span class="cp">#if !defined(CH_USE_MESSAGES) || defined(__DOXYGEN__)</span>
</div><div id="l243" class="code_block"><span class="cp">#define CH_USE_MESSAGES                 TRUE</span>
</div><div id="l244" class="code_block"><span class="cp">#endif</span>
</div><div id="l245" class="code_block">
</div><div id="l246" class="code_block"><span class="cm">/**</span>
</div><div id="l247" class="code_block"><span class="cm"> * @brief   Synchronous Messages queuing mode.</span>
</div><div id="l248" class="code_block"><span class="cm"> * @details If enabled then messages are served by priority rather than in</span>
</div><div id="l249" class="code_block"><span class="cm"> *          FIFO order.</span>
</div><div id="l250" class="code_block"><span class="cm"> *</span>
</div><div id="l251" class="code_block"><span class="cm"> * @note    The default is @p FALSE. Enable this if you have special requirements.</span>
</div><div id="l252" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_MESSAGES.</span>
</div><div id="l253" class="code_block"><span class="cm"> */</span>
</div><div id="l254" class="code_block"><span class="cp">#if !defined(CH_USE_MESSAGES_PRIORITY) || defined(__DOXYGEN__)</span>
</div><div id="l255" class="code_block"><span class="cp">#define CH_USE_MESSAGES_PRIORITY        FALSE</span>
</div><div id="l256" class="code_block"><span class="cp">#endif</span>
</div><div id="l257" class="code_block">
</div><div id="l258" class="code_block"><span class="cm">/**</span>
</div><div id="l259" class="code_block"><span class="cm"> * @brief   Mailboxes APIs.</span>
</div><div id="l260" class="code_block"><span class="cm"> * @details If enabled then the asynchronous messages (mailboxes) APIs are</span>
</div><div id="l261" class="code_block"><span class="cm"> *          included in the kernel.</span>
</div><div id="l262" class="code_block"><span class="cm"> *</span>
</div><div id="l263" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l264" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_SEMAPHORES.</span>
</div><div id="l265" class="code_block"><span class="cm"> */</span>
</div><div id="l266" class="code_block"><span class="cp">#if !defined(CH_USE_MAILBOXES) || defined(__DOXYGEN__)</span>
</div><div id="l267" class="code_block"><span class="cp">#define CH_USE_MAILBOXES                TRUE</span>
</div><div id="l268" class="code_block"><span class="cp">#endif</span>
</div><div id="l269" class="code_block">
</div><div id="l270" class="code_block"><span class="cm">/**</span>
</div><div id="l271" class="code_block"><span class="cm"> * @brief   I/O Queues APIs.</span>
</div><div id="l272" class="code_block"><span class="cm"> * @details If enabled then the I/O queues APIs are included in the kernel.</span>
</div><div id="l273" class="code_block"><span class="cm"> *</span>
</div><div id="l274" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l275" class="code_block"><span class="cm"> */</span>
</div><div id="l276" class="code_block"><span class="cp">#if !defined(CH_USE_QUEUES) || defined(__DOXYGEN__)</span>
</div><div id="l277" class="code_block"><span class="cp">#define CH_USE_QUEUES                   TRUE</span>
</div><div id="l278" class="code_block"><span class="cp">#endif</span>
</div><div id="l279" class="code_block">
</div><div id="l280" class="code_block"><span class="cm">/**</span>
</div><div id="l281" class="code_block"><span class="cm"> * @brief   Core Memory Manager APIs.</span>
</div><div id="l282" class="code_block"><span class="cm"> * @details If enabled then the core memory manager APIs are included</span>
</div><div id="l283" class="code_block"><span class="cm"> *          in the kernel.</span>
</div><div id="l284" class="code_block"><span class="cm"> *</span>
</div><div id="l285" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l286" class="code_block"><span class="cm"> */</span>
</div><div id="l287" class="code_block"><span class="cp">#if !defined(CH_USE_MEMCORE) || defined(__DOXYGEN__)</span>
</div><div id="l288" class="code_block"><span class="cp">#define CH_USE_MEMCORE                  FALSE</span>
</div><div id="l289" class="code_block"><span class="cp">#endif</span>
</div><div id="l290" class="code_block">
</div><div id="l291" class="code_block"><span class="cm">/**</span>
</div><div id="l292" class="code_block"><span class="cm"> * @brief   Heap Allocator APIs.</span>
</div><div id="l293" class="code_block"><span class="cm"> * @details If enabled then the memory heap allocator APIs are included</span>
</div><div id="l294" class="code_block"><span class="cm"> *          in the kernel.</span>
</div><div id="l295" class="code_block"><span class="cm"> *</span>
</div><div id="l296" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l297" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_MEMCORE and either @p CH_USE_MUTEXES or</span>
</div><div id="l298" class="code_block"><span class="cm"> *          @p CH_USE_SEMAPHORES.</span>
</div><div id="l299" class="code_block"><span class="cm"> * @note    Mutexes are recommended.</span>
</div><div id="l300" class="code_block"><span class="cm"> */</span>
</div><div id="l301" class="code_block"><span class="cp">#if !defined(CH_USE_HEAP) || defined(__DOXYGEN__)</span>
</div><div id="l302" class="code_block"><span class="cp">#define CH_USE_HEAP                     FALSE</span>
</div><div id="l303" class="code_block"><span class="cp">#endif</span>
</div><div id="l304" class="code_block">
</div><div id="l305" class="code_block"><span class="cm">/**</span>
</div><div id="l306" class="code_block"><span class="cm"> * @brief   C-runtime allocator.</span>
</div><div id="l307" class="code_block"><span class="cm"> * @details If enabled the the heap allocator APIs just wrap the C-runtime</span>
</div><div id="l308" class="code_block"><span class="cm"> *          @p malloc() and @p free() functions.</span>
</div><div id="l309" class="code_block"><span class="cm"> *</span>
</div><div id="l310" class="code_block"><span class="cm"> * @note    The default is @p FALSE.</span>
</div><div id="l311" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_HEAP.</span>
</div><div id="l312" class="code_block"><span class="cm"> * @note    The C-runtime may or may not require @p CH_USE_MEMCORE, see the</span>
</div><div id="l313" class="code_block"><span class="cm"> *          appropriate documentation.</span>
</div><div id="l314" class="code_block"><span class="cm"> */</span>
</div><div id="l315" class="code_block"><span class="cp">#if !defined(CH_USE_MALLOC_HEAP) || defined(__DOXYGEN__)</span>
</div><div id="l316" class="code_block"><span class="cp">#define CH_USE_MALLOC_HEAP              FALSE</span>
</div><div id="l317" class="code_block"><span class="cp">#endif</span>
</div><div id="l318" class="code_block">
</div><div id="l319" class="code_block"><span class="cm">/**</span>
</div><div id="l320" class="code_block"><span class="cm"> * @brief   Memory Pools Allocator APIs.</span>
</div><div id="l321" class="code_block"><span class="cm"> * @details If enabled then the memory pools allocator APIs are included</span>
</div><div id="l322" class="code_block"><span class="cm"> *          in the kernel.</span>
</div><div id="l323" class="code_block"><span class="cm"> *</span>
</div><div id="l324" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l325" class="code_block"><span class="cm"> */</span>
</div><div id="l326" class="code_block"><span class="cp">#if !defined(CH_USE_MEMPOOLS) || defined(__DOXYGEN__)</span>
</div><div id="l327" class="code_block"><span class="cp">#define CH_USE_MEMPOOLS                 FALSE</span>
</div><div id="l328" class="code_block"><span class="cp">#endif</span>
</div><div id="l329" class="code_block">
</div><div id="l330" class="code_block"><span class="cm">/**</span>
</div><div id="l331" class="code_block"><span class="cm"> * @brief   Dynamic Threads APIs.</span>
</div><div id="l332" class="code_block"><span class="cm"> * @details If enabled then the dynamic threads creation APIs are included</span>
</div><div id="l333" class="code_block"><span class="cm"> *          in the kernel.</span>
</div><div id="l334" class="code_block"><span class="cm"> *</span>
</div><div id="l335" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l336" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_WAITEXIT.</span>
</div><div id="l337" class="code_block"><span class="cm"> * @note    Requires @p CH_USE_HEAP and/or @p CH_USE_MEMPOOLS.</span>
</div><div id="l338" class="code_block"><span class="cm"> */</span>
</div><div id="l339" class="code_block"><span class="cp">#if !defined(CH_USE_DYNAMIC) || defined(__DOXYGEN__)</span>
</div><div id="l340" class="code_block"><span class="cp">#define CH_USE_DYNAMIC                  FALSE</span>
</div><div id="l341" class="code_block"><span class="cp">#endif</span>
</div><div id="l342" class="code_block">
</div><div id="l343" class="code_block"><span class="cm">/** @} */</span>
</div><div id="l344" class="code_block">
</div><div id="l345" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l346" class="code_block"><span class="cm">/**</span>
</div><div id="l347" class="code_block"><span class="cm"> * @name Debug options</span>
</div><div id="l348" class="code_block"><span class="cm"> * @{</span>
</div><div id="l349" class="code_block"><span class="cm"> */</span>
</div><div id="l350" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l351" class="code_block">
</div><div id="l352" class="code_block"><span class="cm">/**</span>
</div><div id="l353" class="code_block"><span class="cm"> * @brief   Debug option, system state check.</span>
</div><div id="l354" class="code_block"><span class="cm"> * @details If enabled the correct call protocol for system APIs is checked</span>
</div><div id="l355" class="code_block"><span class="cm"> *          at runtime.</span>
</div><div id="l356" class="code_block"><span class="cm"> *</span>
</div><div id="l357" class="code_block"><span class="cm"> * @note    The default is @p FALSE.</span>
</div><div id="l358" class="code_block"><span class="cm"> */</span>
</div><div id="l359" class="code_block"><span class="cp">#if !defined(CH_DBG_SYSTEM_STATE_CHECK) || defined(__DOXYGEN__)</span>
</div><div id="l360" class="code_block"><span class="cp">#define CH_DBG_SYSTEM_STATE_CHECK       FALSE</span>
</div><div id="l361" class="code_block"><span class="cp">#endif</span>
</div><div id="l362" class="code_block">
</div><div id="l363" class="code_block"><span class="cm">/**</span>
</div><div id="l364" class="code_block"><span class="cm"> * @brief   Debug option, parameters checks.</span>
</div><div id="l365" class="code_block"><span class="cm"> * @details If enabled then the checks on the API functions input</span>
</div><div id="l366" class="code_block"><span class="cm"> *          parameters are activated.</span>
</div><div id="l367" class="code_block"><span class="cm"> *</span>
</div><div id="l368" class="code_block"><span class="cm"> * @note    The default is @p FALSE.</span>
</div><div id="l369" class="code_block"><span class="cm"> */</span>
</div><div id="l370" class="code_block"><span class="cp">#if !defined(CH_DBG_ENABLE_CHECKS) || defined(__DOXYGEN__)</span>
</div><div id="l371" class="code_block"><span class="cp">#define CH_DBG_ENABLE_CHECKS            FALSE</span>
</div><div id="l372" class="code_block"><span class="cp">#endif</span>
</div><div id="l373" class="code_block">
</div><div id="l374" class="code_block"><span class="cm">/**</span>
</div><div id="l375" class="code_block"><span class="cm"> * @brief   Debug option, consistency checks.</span>
</div><div id="l376" class="code_block"><span class="cm"> * @details If enabled then all the assertions in the kernel code are</span>
</div><div id="l377" class="code_block"><span class="cm"> *          activated. This includes consistency checks inside the kernel,</span>
</div><div id="l378" class="code_block"><span class="cm"> *          runtime anomalies and port-defined checks.</span>
</div><div id="l379" class="code_block"><span class="cm"> *</span>
</div><div id="l380" class="code_block"><span class="cm"> * @note    The default is @p FALSE.</span>
</div><div id="l381" class="code_block"><span class="cm"> */</span>
</div><div id="l382" class="code_block"><span class="cp">#if !defined(CH_DBG_ENABLE_ASSERTS) || defined(__DOXYGEN__)</span>
</div><div id="l383" class="code_block"><span class="cp">#define CH_DBG_ENABLE_ASSERTS           FALSE</span>
</div><div id="l384" class="code_block"><span class="cp">#endif</span>
</div><div id="l385" class="code_block">
</div><div id="l386" class="code_block"><span class="cm">/**</span>
</div><div id="l387" class="code_block"><span class="cm"> * @brief   Debug option, trace buffer.</span>
</div><div id="l388" class="code_block"><span class="cm"> * @details If enabled then the context switch circular trace buffer is</span>
</div><div id="l389" class="code_block"><span class="cm"> *          activated.</span>
</div><div id="l390" class="code_block"><span class="cm"> *</span>
</div><div id="l391" class="code_block"><span class="cm"> * @note    The default is @p FALSE.</span>
</div><div id="l392" class="code_block"><span class="cm"> */</span>
</div><div id="l393" class="code_block"><span class="cp">#if !defined(CH_DBG_ENABLE_TRACE) || defined(__DOXYGEN__)</span>
</div><div id="l394" class="code_block"><span class="cp">#define CH_DBG_ENABLE_TRACE             FALSE</span>
</div><div id="l395" class="code_block"><span class="cp">#endif</span>
</div><div id="l396" class="code_block">
</div><div id="l397" class="code_block"><span class="cm">/**</span>
</div><div id="l398" class="code_block"><span class="cm"> * @brief   Debug option, stack checks.</span>
</div><div id="l399" class="code_block"><span class="cm"> * @details If enabled then a runtime stack check is performed.</span>
</div><div id="l400" class="code_block"><span class="cm"> *</span>
</div><div id="l401" class="code_block"><span class="cm"> * @note    The default is @p FALSE.</span>
</div><div id="l402" class="code_block"><span class="cm"> * @note    The stack check is performed in a architecture/port dependent way.</span>
</div><div id="l403" class="code_block"><span class="cm"> *          It may not be implemented or some ports.</span>
</div><div id="l404" class="code_block"><span class="cm"> * @note    The default failure mode is to halt the system with the global</span>
</div><div id="l405" class="code_block"><span class="cm"> *          @p panic_msg variable set to @p NULL.</span>
</div><div id="l406" class="code_block"><span class="cm"> */</span>
</div><div id="l407" class="code_block"><span class="cp">#if !defined(CH_DBG_ENABLE_STACK_CHECK) || defined(__DOXYGEN__)</span>
</div><div id="l408" class="code_block"><span class="cp">#define CH_DBG_ENABLE_STACK_CHECK       FALSE</span>
</div><div id="l409" class="code_block"><span class="cp">#endif</span>
</div><div id="l410" class="code_block">
</div><div id="l411" class="code_block"><span class="cm">/**</span>
</div><div id="l412" class="code_block"><span class="cm"> * @brief   Debug option, stacks initialization.</span>
</div><div id="l413" class="code_block"><span class="cm"> * @details If enabled then the threads working area is filled with a byte</span>
</div><div id="l414" class="code_block"><span class="cm"> *          value when a thread is created. This can be useful for the</span>
</div><div id="l415" class="code_block"><span class="cm"> *          runtime measurement of the used stack.</span>
</div><div id="l416" class="code_block"><span class="cm"> *</span>
</div><div id="l417" class="code_block"><span class="cm"> * @note    The default is @p FALSE.</span>
</div><div id="l418" class="code_block"><span class="cm"> */</span>
</div><div id="l419" class="code_block"><span class="cp">#if !defined(CH_DBG_FILL_THREADS) || defined(__DOXYGEN__)</span>
</div><div id="l420" class="code_block"><span class="cp">#define CH_DBG_FILL_THREADS             FALSE</span>
</div><div id="l421" class="code_block"><span class="cp">#endif</span>
</div><div id="l422" class="code_block">
</div><div id="l423" class="code_block"><span class="cm">/**</span>
</div><div id="l424" class="code_block"><span class="cm"> * @brief   Debug option, threads profiling.</span>
</div><div id="l425" class="code_block"><span class="cm"> * @details If enabled then a field is added to the @p Thread structure that</span>
</div><div id="l426" class="code_block"><span class="cm"> *          counts the system ticks occurred while executing the thread.</span>
</div><div id="l427" class="code_block"><span class="cm"> *</span>
</div><div id="l428" class="code_block"><span class="cm"> * @note    The default is @p TRUE.</span>
</div><div id="l429" class="code_block"><span class="cm"> * @note    This debug option is defaulted to TRUE because it is required by</span>
</div><div id="l430" class="code_block"><span class="cm"> *          some test cases into the test suite.</span>
</div><div id="l431" class="code_block"><span class="cm"> */</span>
</div><div id="l432" class="code_block"><span class="cp">#if !defined(CH_DBG_THREADS_PROFILING) || defined(__DOXYGEN__)</span>
</div><div id="l433" class="code_block"><span class="cp">#define CH_DBG_THREADS_PROFILING        TRUE</span>
</div><div id="l434" class="code_block"><span class="cp">#endif</span>
</div><div id="l435" class="code_block">
</div><div id="l436" class="code_block"><span class="cm">/** @} */</span>
</div><div id="l437" class="code_block">
</div><div id="l438" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l439" class="code_block"><span class="cm">/**</span>
</div><div id="l440" class="code_block"><span class="cm"> * @name Kernel hooks</span>
</div><div id="l441" class="code_block"><span class="cm"> * @{</span>
</div><div id="l442" class="code_block"><span class="cm"> */</span>
</div><div id="l443" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l444" class="code_block">
</div><div id="l445" class="code_block"><span class="cm">/**</span>
</div><div id="l446" class="code_block"><span class="cm"> * @brief   Threads descriptor structure extension.</span>
</div><div id="l447" class="code_block"><span class="cm"> * @details User fields added to the end of the @p Thread structure.</span>
</div><div id="l448" class="code_block"><span class="cm"> */</span>
</div><div id="l449" class="code_block"><span class="cp">#if !defined(THREAD_EXT_FIELDS) || defined(__DOXYGEN__)</span>
</div><div id="l450" class="code_block"><span class="cp">#define THREAD_EXT_FIELDS                                                   \</span>
</div><div id="l451" class="code_block"><span class="cp">  </span><span class="cm">/* Add threads custom fields here.*/</span><span class="cp"></span>
</div><div id="l452" class="code_block"><span class="cp">#endif</span>
</div><div id="l453" class="code_block">
</div><div id="l454" class="code_block"><span class="cm">/**</span>
</div><div id="l455" class="code_block"><span class="cm"> * @brief   Threads initialization hook.</span>
</div><div id="l456" class="code_block"><span class="cm"> * @details User initialization code added to the @p chThdInit() API.</span>
</div><div id="l457" class="code_block"><span class="cm"> *</span>
</div><div id="l458" class="code_block"><span class="cm"> * @note    It is invoked from within @p chThdInit() and implicitly from all</span>
</div><div id="l459" class="code_block"><span class="cm"> *          the threads creation APIs.</span>
</div><div id="l460" class="code_block"><span class="cm"> */</span>
</div><div id="l461" class="code_block"><span class="cp">#if !defined(THREAD_EXT_INIT_HOOK) || defined(__DOXYGEN__)</span>
</div><div id="l462" class="code_block"><span class="cp">#define THREAD_EXT_INIT_HOOK(tp) {                                          \</span>
</div><div id="l463" class="code_block"><span class="cp">  </span><span class="cm">/* Add threads initialization code here.*/</span><span class="cp">                                \</span>
</div><div id="l464" class="code_block"><span class="cp">}</span>
</div><div id="l465" class="code_block"><span class="cp">#endif</span>
</div><div id="l466" class="code_block">
</div><div id="l467" class="code_block"><span class="cm">/**</span>
</div><div id="l468" class="code_block"><span class="cm"> * @brief   Threads finalization hook.</span>
</div><div id="l469" class="code_block"><span class="cm"> * @details User finalization code added to the @p chThdExit() API.</span>
</div><div id="l470" class="code_block"><span class="cm"> *</span>
</div><div id="l471" class="code_block"><span class="cm"> * @note    It is inserted into lock zone.</span>
</div><div id="l472" class="code_block"><span class="cm"> * @note    It is also invoked when the threads simply return in order to</span>
</div><div id="l473" class="code_block"><span class="cm"> *          terminate.</span>
</div><div id="l474" class="code_block"><span class="cm"> */</span>
</div><div id="l475" class="code_block"><span class="cp">#if !defined(THREAD_EXT_EXIT_HOOK) || defined(__DOXYGEN__)</span>
</div><div id="l476" class="code_block"><span class="cp">#define THREAD_EXT_EXIT_HOOK(tp) {                                          \</span>
</div><div id="l477" class="code_block"><span class="cp">  </span><span class="cm">/* Add threads finalization code here.*/</span><span class="cp">                                  \</span>
</div><div id="l478" class="code_block"><span class="cp">}</span>
</div><div id="l479" class="code_block"><span class="cp">#endif</span>
</div><div id="l480" class="code_block">
</div><div id="l481" class="code_block"><span class="cm">/**</span>
</div><div id="l482" class="code_block"><span class="cm"> * @brief   Context switch hook.</span>
</div><div id="l483" class="code_block"><span class="cm"> * @details This hook is invoked just before switching between threads.</span>
</div><div id="l484" class="code_block"><span class="cm"> */</span>
</div><div id="l485" class="code_block"><span class="cp">#if !defined(THREAD_CONTEXT_SWITCH_HOOK) || defined(__DOXYGEN__)</span>
</div><div id="l486" class="code_block"><span class="cp">#define THREAD_CONTEXT_SWITCH_HOOK(ntp, otp) {                              \</span>
</div><div id="l487" class="code_block"><span class="cp">  </span><span class="cm">/* System halt code here.*/</span><span class="cp">                                               \</span>
</div><div id="l488" class="code_block"><span class="cp">}</span>
</div><div id="l489" class="code_block"><span class="cp">#endif</span>
</div><div id="l490" class="code_block">
</div><div id="l491" class="code_block"><span class="cm">/**</span>
</div><div id="l492" class="code_block"><span class="cm"> * @brief   Idle Loop hook.</span>
</div><div id="l493" class="code_block"><span class="cm"> * @details This hook is continuously invoked by the idle thread loop.</span>
</div><div id="l494" class="code_block"><span class="cm"> */</span>
</div><div id="l495" class="code_block"><span class="cp">#if !defined(IDLE_LOOP_HOOK) || defined(__DOXYGEN__)</span>
</div><div id="l496" class="code_block"><span class="cp">#define IDLE_LOOP_HOOK() {                                                  \</span>
</div><div id="l497" class="code_block"><span class="cp">  </span><span class="cm">/* Idle loop code here.*/</span><span class="cp">                                                 \</span>
</div><div id="l498" class="code_block"><span class="cp">}</span>
</div><div id="l499" class="code_block"><span class="cp">#endif</span>
</div><div id="l500" class="code_block">
</div><div id="l501" class="code_block"><span class="cm">/**</span>
</div><div id="l502" class="code_block"><span class="cm"> * @brief   System tick event hook.</span>
</div><div id="l503" class="code_block"><span class="cm"> * @details This hook is invoked in the system tick handler immediately</span>
</div><div id="l504" class="code_block"><span class="cm"> *          after processing the virtual timers queue.</span>
</div><div id="l505" class="code_block"><span class="cm"> */</span>
</div><div id="l506" class="code_block"><span class="cp">#if !defined(SYSTEM_TICK_EVENT_HOOK) || defined(__DOXYGEN__)</span>
</div><div id="l507" class="code_block"><span class="cp">#define SYSTEM_TICK_EVENT_HOOK() {                                          \</span>
</div><div id="l508" class="code_block"><span class="cp">  </span><span class="cm">/* System tick event code here.*/</span><span class="cp">                                         \</span>
</div><div id="l509" class="code_block"><span class="cp">}</span>
</div><div id="l510" class="code_block"><span class="cp">#endif</span>
</div><div id="l511" class="code_block">
</div><div id="l512" class="code_block"><span class="cm">/**</span>
</div><div id="l513" class="code_block"><span class="cm"> * @brief   System halt hook.</span>
</div><div id="l514" class="code_block"><span class="cm"> * @details This hook is invoked in case to a system halting error before</span>
</div><div id="l515" class="code_block"><span class="cm"> *          the system is halted.</span>
</div><div id="l516" class="code_block"><span class="cm"> */</span>
</div><div id="l517" class="code_block"><span class="cp">#if !defined(SYSTEM_HALT_HOOK) || defined(__DOXYGEN__)</span>
</div><div id="l518" class="code_block"><span class="cp">#define SYSTEM_HALT_HOOK() {                                                \</span>
</div><div id="l519" class="code_block"><span class="cp">  </span><span class="cm">/* System halt code here.*/</span><span class="cp">                                               \</span>
</div><div id="l520" class="code_block"><span class="cp">}</span>
</div><div id="l521" class="code_block"><span class="cp">#endif</span>
</div><div id="l522" class="code_block">
</div><div id="l523" class="code_block"><span class="cm">/** @} */</span>
</div><div id="l524" class="code_block">
</div><div id="l525" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l526" class="code_block"><span class="cm">/* Port-specific settings (override port settings defaulted in chcore.h).    */</span>
</div><div id="l527" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l528" class="code_block">
</div><div id="l529" class="code_block"><span class="cp">#endif  </span><span class="cm">/* _CHCONF_H_ */</span><span class="cp"></span>
</div><div id="l530" class="code_block">
</div><div id="l531" class="code_block"><span class="cm">/** @} */</span>
</div></pre></div>
</td></tr></table>
      
    </div>
  

                </div>
                
                
            </div>
        
    </div>
</section>
  
<footer id="site-footer">
    <div class="wrapper">
        <nav>
            <h5>SourceForge</h5>
            <a href="/about">About</a>
            <a href="/blog/category/sitestatus/">Site Status</a>
            <a href="http://twitter.com/sfnet_ops">@sfnet_ops</a>
            <a id="allura-notice" href="http://allura.apache.org/">
                <p>Powered by</p>
                <p>Apache Allura™</p>
                <img src="http://a.fsdn.com/allura/nf/1452191356/_ew_/theme/sftheme/images/sftheme/logo-black-svg_g.png" />
            </a>
        </nav>
        <nav>
            <h5>Find and Develop Software</h5>
            <a href="/create/">Create a Project</a>
            <a href="/directory/">Software Directory</a>
            <a href="/top">Top Downloaded Projects</a>
        </nav>
        <nav>
            <h5>Community</h5>
            <a href="/blog/">Blog</a>
            <a href="http://twitter.com/sourceforge">@sourceforge</a>
            <a href="/jobs?source=footer">Job Board</a>
            <a href="http://library.slashdotmedia.com/?source=sfnet_footer">Resources</a>
        </nav>
        <nav>
            <h5>Help</h5>
            <a href="http://p.sf.net/sourceforge/docs">Site Documentation</a>
            <a href="/support">Support Request</a>
            <a href="http://p.sf.net/sourceforge/irc">Real-Time Support</a>
        </nav>
    </div>
</footer>
<footer id="site-copyright-footer">
    <div class="wrapper">
        <div id="copyright">
            &copy; 2016 Slashdot Media. All Rights Reserved.<br />
            <div id="dhi-icon"><span class="logo-DHI-alt"></span></div>
            <div id="service-text"><div class="smalltext"> SourceForge is a <a href="http://www.dhigroupinc.com" target="_blank">DHI service</a></div></div>
        </div>
        <nav>
            <a href="http://slashdotmedia.com/terms-of-use">Terms</a>
            <a href="http://slashdotmedia.com/privacy-statement/">Privacy</a>
            <span id='teconsent'></span>
            <a href="http://slashdotmedia.com/opt-out-choices">Opt Out Choices</a>
            <a href="http://slashdotmedia.com">Advertise</a>
        </nav>
    </div>
</footer>
<div id="messages">
    
</div>


    <!-- ew:body_js -->


    <script type="text/javascript" src="http://a.fsdn.com/allura/nf/1452191356/_ew_/_slim/js?href=allura%2Fjs%2Fjquery.notify.js%3Ballura%2Fjs%2Fjquery.tooltipster.js%3Ballura%2Fjs%2Fmodernizr.js%3Ballura%2Fjs%2Fsylvester.js%3Ballura%2Fjs%2Fpb.transformie.min.js%3Ballura%2Fjs%2Fallura-base.js%3Ballura%2Fjs%2Fbuild%2Ftranspiled.js%3Btheme%2Fsftheme%2Fjs%2Fsftheme%2Fheader.js%3Ballura%2Fjs%2Fmaximize-content.js"></script>

    
<!-- /ew:body_js -->



    <!-- ew:body_js_tail -->


    
<!-- /ew:body_js_tail -->




<script type="text/javascript">(function() {
  $('#access_urls .btn').click(function(evt){
    evt.preventDefault();
    var parent = $(this).parents('.btn-bar');
    $(parent).find('input').val($(this).attr('data-url'));
    $(parent).find('span').text($(this).attr('title')+' access');
    $(this).parent().children('.btn').removeClass('active');
    $(this).addClass('active');
  });
  $('#access_urls .btn').first().click();

  
  var repo_status = document.getElementById('repo_status');
  // The repo_status div will only be present if repo.status != 'ready'
  if (repo_status) {
    $('.spinner').show()
    function check_status() {
        $.get('/p/chibios/svn/status', function(data) {
            if (data.status === 'ready') {
                window.clearInterval(status_checker);
                $('.spinner').hide()
                $('#repo_status h2').html('Repo status: ready. <a href=".">Click here to refresh this page.</a>');
            }
            else {
                $('#repo_status h2 span').html(data.status);
            }
        });
    }
    // Check repo status every 15 seconds
    var status_checker = window.setInterval(check_status, 15000);
    
  }
}());
</script>

<script type="text/javascript">(function() {
  $(window).bind('hashchange', function(e) {
    var hash = window.location.hash.substring(1);
	if ('originalEvent' in e && 'oldURL' in e.originalEvent) {
      $('#' + e.originalEvent.oldURL.split('#')[1]).css('background-color', 'transparent');
	}
    if (hash !== '' && hash.substring(0, 1) === 'l' && !isNaN(hash.substring(1))) {
      $('#' + hash).css('background-color', '#ffff99');
    }
  }).trigger('hashchange');

  var clicks = 0;
  $('.code_block').each(function(index, element) {
    $(element).bind('click', function() {
      // Trick to ignore double and triple clicks
      clicks++;
      if (clicks == 1) {
        setTimeout(function() {
          if (clicks == 1) {
            var hash = window.location.hash.substring(1);
            if (hash !== '' && hash.substring(0, 1) === 'l' && !isNaN(hash.substring(1))) {
              $('#' + hash).css('background-color', 'transparent');
            }
            $(element).css('background-color', '#ffff99');
            window.location.href = '#' + $(element).attr('id');
          };
          clicks = 0;
        }, 500);
      };
    });
  });
}());
</script>


    


    <!-- Google Code for Remarketing tag -->
    <!-- Remarketing tags may not be associated with personally identifiable information or placed on pages related to sensitive categories. For instructions on adding this tag and more information on the above requirements, read the setup guide: google.com/ads/remarketingsetup -->
    <script type="text/javascript">
        /* <![CDATA[ */
        var google_conversion_id = 1002083962;
        var google_conversion_label = "G_uGCOaBlAQQ-qzq3QM";
        var google_custom_params = window.google_tag_params;
        var google_remarketing_only = true;
        /* ]]> */
    </script>
    <script type="text/javascript" src="//www.googleadservices.com/pagead/conversion.js"> </script>
    <script type="text/javascript" src='//consent-st.truste.com/get?name=notice.js&domain=slashdot.org&c=teconsent&text=true'></script>
    <noscript>
      <div style="display:inline;">
        <img height="1" width="1" style="border-style:none;" alt="" src="//googleads.g.doubleclick.net/pagead/viewthroughconversion/1002083962/?value=0&amp;label=G_uGCOaBlAQQ-qzq3QM&amp;guid=ON&amp;script=0"/>
      </div>
    </noscript>

     
      
        <script> SF.SimplifiedCookieNotice().init(); </script>
      

<script>
    $(document).ready(function () {
        $(".tooltip").tooltipster({
            animation: 'fade',
            delay: 200,
            theme: 'tooltipster-light',
            trigger: 'hover',
            position: 'right',
            iconCloning: false,
            maxWidth: 300
        }).focus(function () {
            $(this).tooltipster('show');
        }).blur(function () {
            $(this).tooltipster('hide');
        });
    })
</script>
</body>
</html>