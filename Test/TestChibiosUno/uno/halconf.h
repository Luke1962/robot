<!DOCTYPE html>
<!-- Server: sfn-web-9 -->


    















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
  /trunk_old/demos/AVR-Arduino-GCC/halconf.h
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

    <style>.XBvtnuJahQaKRpNslXTmntLNjI {
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
    
  
 halconf.h

                    <!-- actions -->
                    <small>
                        

    
    <a class="icon" href="#" id="maximize-content" title="Maximize"><i class="fa fa-expand"></i>&nbsp;Maximize</a>
    <a class="icon" href="#" id="restore-content" title="Restore"><i class="fa fa-compress"></i>&nbsp;Restore</a>
<a class="icon" href="/p/chibios/svn/8742/log/?path=/trunk_old/demos/AVR-Arduino-GCC/halconf.h" title="History"><i class="fa fa-calendar"></i>&nbsp;History</a>

                    </small>
                    <!-- /actions -->
                </h2>
                
                <div>
                    
  

                    
  
    <p><a href="?format=raw">Download this file</a></p>
    <div class="clip grid-19 codebrowser">
      <h3>
        320 lines (270 with data), 9.8 kB
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
319</pre></div></td><td class="code"><div class="codehilite"><pre><div id="l1" class="code_block"><span class="cm">/*</span>
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
</div><div id="l18" class="code_block"><span class="cm"> * @file    templates/halconf.h</span>
</div><div id="l19" class="code_block"><span class="cm"> * @brief   HAL configuration header.</span>
</div><div id="l20" class="code_block"><span class="cm"> * @details HAL configuration file, this file allows to enable or disable the</span>
</div><div id="l21" class="code_block"><span class="cm"> *          various device drivers from your application. You may also use</span>
</div><div id="l22" class="code_block"><span class="cm"> *          this file in order to override the device drivers default settings.</span>
</div><div id="l23" class="code_block"><span class="cm"> *</span>
</div><div id="l24" class="code_block"><span class="cm"> * @addtogroup HAL_CONF</span>
</div><div id="l25" class="code_block"><span class="cm"> * @{</span>
</div><div id="l26" class="code_block"><span class="cm"> */</span>
</div><div id="l27" class="code_block">
</div><div id="l28" class="code_block"><span class="cp">#ifndef _HALCONF_H_</span>
</div><div id="l29" class="code_block"><span class="cp">#define _HALCONF_H_</span>
</div><div id="l30" class="code_block">
</div><div id="l31" class="code_block"><span class="cp">#include &quot;mcuconf.h&quot;</span>
</div><div id="l32" class="code_block">
</div><div id="l33" class="code_block"><span class="cm">/**</span>
</div><div id="l34" class="code_block"><span class="cm"> * @brief   Enables the TM subsystem.</span>
</div><div id="l35" class="code_block"><span class="cm"> */</span>
</div><div id="l36" class="code_block"><span class="cp">#if !defined(HAL_USE_TM) || defined(__DOXYGEN__)</span>
</div><div id="l37" class="code_block"><span class="cp">#define HAL_USE_TM                  FALSE</span>
</div><div id="l38" class="code_block"><span class="cp">#endif</span>
</div><div id="l39" class="code_block">
</div><div id="l40" class="code_block"><span class="cm">/**</span>
</div><div id="l41" class="code_block"><span class="cm"> * @brief   Enables the PAL subsystem.</span>
</div><div id="l42" class="code_block"><span class="cm"> */</span>
</div><div id="l43" class="code_block"><span class="cp">#if !defined(HAL_USE_PAL) || defined(__DOXYGEN__)</span>
</div><div id="l44" class="code_block"><span class="cp">#define HAL_USE_PAL                 TRUE</span>
</div><div id="l45" class="code_block"><span class="cp">#endif</span>
</div><div id="l46" class="code_block">
</div><div id="l47" class="code_block"><span class="cm">/**</span>
</div><div id="l48" class="code_block"><span class="cm"> * @brief   Enables the ADC subsystem.</span>
</div><div id="l49" class="code_block"><span class="cm"> */</span>
</div><div id="l50" class="code_block"><span class="cp">#if !defined(HAL_USE_ADC) || defined(__DOXYGEN__)</span>
</div><div id="l51" class="code_block"><span class="cp">#define HAL_USE_ADC                 FALSE</span>
</div><div id="l52" class="code_block"><span class="cp">#endif</span>
</div><div id="l53" class="code_block">
</div><div id="l54" class="code_block"><span class="cm">/**</span>
</div><div id="l55" class="code_block"><span class="cm"> * @brief   Enables the DAC subsystem.</span>
</div><div id="l56" class="code_block"><span class="cm"> */</span>
</div><div id="l57" class="code_block"><span class="cp">#if !defined(HAL_USE_DAC) || defined(__DOXYGEN__)</span>
</div><div id="l58" class="code_block"><span class="cp">#define HAL_USE_DAC                 FALSE</span>
</div><div id="l59" class="code_block"><span class="cp">#endif</span>
</div><div id="l60" class="code_block">
</div><div id="l61" class="code_block"><span class="cm">/**</span>
</div><div id="l62" class="code_block"><span class="cm"> * @brief   Enables the CAN subsystem.</span>
</div><div id="l63" class="code_block"><span class="cm"> */</span>
</div><div id="l64" class="code_block"><span class="cp">#if !defined(HAL_USE_CAN) || defined(__DOXYGEN__)</span>
</div><div id="l65" class="code_block"><span class="cp">#define HAL_USE_CAN                 FALSE</span>
</div><div id="l66" class="code_block"><span class="cp">#endif</span>
</div><div id="l67" class="code_block">
</div><div id="l68" class="code_block"><span class="cm">/**</span>
</div><div id="l69" class="code_block"><span class="cm"> * @brief   Enables the EXT subsystem.</span>
</div><div id="l70" class="code_block"><span class="cm"> */</span>
</div><div id="l71" class="code_block"><span class="cp">#if !defined(HAL_USE_EXT) || defined(__DOXYGEN__)</span>
</div><div id="l72" class="code_block"><span class="cp">#define HAL_USE_EXT                 FALSE</span>
</div><div id="l73" class="code_block"><span class="cp">#endif</span>
</div><div id="l74" class="code_block">
</div><div id="l75" class="code_block"><span class="cm">/**</span>
</div><div id="l76" class="code_block"><span class="cm"> * @brief   Enables the GPT subsystem.</span>
</div><div id="l77" class="code_block"><span class="cm"> */</span>
</div><div id="l78" class="code_block"><span class="cp">#if !defined(HAL_USE_GPT) || defined(__DOXYGEN__)</span>
</div><div id="l79" class="code_block"><span class="cp">#define HAL_USE_GPT                 FALSE</span>
</div><div id="l80" class="code_block"><span class="cp">#endif</span>
</div><div id="l81" class="code_block">
</div><div id="l82" class="code_block"><span class="cm">/**</span>
</div><div id="l83" class="code_block"><span class="cm"> * @brief   Enables the I2C subsystem.</span>
</div><div id="l84" class="code_block"><span class="cm"> */</span>
</div><div id="l85" class="code_block"><span class="cp">#if !defined(HAL_USE_I2C) || defined(__DOXYGEN__)</span>
</div><div id="l86" class="code_block"><span class="cp">#define HAL_USE_I2C                 FALSE</span>
</div><div id="l87" class="code_block"><span class="cp">#endif</span>
</div><div id="l88" class="code_block">
</div><div id="l89" class="code_block"><span class="cm">/**</span>
</div><div id="l90" class="code_block"><span class="cm"> * @brief   Enables the ICU subsystem.</span>
</div><div id="l91" class="code_block"><span class="cm"> */</span>
</div><div id="l92" class="code_block"><span class="cp">#if !defined(HAL_USE_ICU) || defined(__DOXYGEN__)</span>
</div><div id="l93" class="code_block"><span class="cp">#define HAL_USE_ICU                 FALSE</span>
</div><div id="l94" class="code_block"><span class="cp">#endif</span>
</div><div id="l95" class="code_block">
</div><div id="l96" class="code_block"><span class="cm">/**</span>
</div><div id="l97" class="code_block"><span class="cm"> * @brief   Enables the MAC subsystem.</span>
</div><div id="l98" class="code_block"><span class="cm"> */</span>
</div><div id="l99" class="code_block"><span class="cp">#if !defined(HAL_USE_MAC) || defined(__DOXYGEN__)</span>
</div><div id="l100" class="code_block"><span class="cp">#define HAL_USE_MAC                 FALSE</span>
</div><div id="l101" class="code_block"><span class="cp">#endif</span>
</div><div id="l102" class="code_block">
</div><div id="l103" class="code_block"><span class="cm">/**</span>
</div><div id="l104" class="code_block"><span class="cm"> * @brief   Enables the MMC_SPI subsystem.</span>
</div><div id="l105" class="code_block"><span class="cm"> */</span>
</div><div id="l106" class="code_block"><span class="cp">#if !defined(HAL_USE_MMC_SPI) || defined(__DOXYGEN__)</span>
</div><div id="l107" class="code_block"><span class="cp">#define HAL_USE_MMC_SPI             FALSE</span>
</div><div id="l108" class="code_block"><span class="cp">#endif</span>
</div><div id="l109" class="code_block">
</div><div id="l110" class="code_block"><span class="cm">/**</span>
</div><div id="l111" class="code_block"><span class="cm"> * @brief   Enables the PWM subsystem.</span>
</div><div id="l112" class="code_block"><span class="cm"> */</span>
</div><div id="l113" class="code_block"><span class="cp">#if !defined(HAL_USE_PWM) || defined(__DOXYGEN__)</span>
</div><div id="l114" class="code_block"><span class="cp">#define HAL_USE_PWM                 FALSE</span>
</div><div id="l115" class="code_block"><span class="cp">#endif</span>
</div><div id="l116" class="code_block">
</div><div id="l117" class="code_block"><span class="cm">/**</span>
</div><div id="l118" class="code_block"><span class="cm"> * @brief   Enables the RTC subsystem.</span>
</div><div id="l119" class="code_block"><span class="cm"> */</span>
</div><div id="l120" class="code_block"><span class="cp">#if !defined(HAL_USE_RTC) || defined(__DOXYGEN__)</span>
</div><div id="l121" class="code_block"><span class="cp">#define HAL_USE_RTC                 FALSE</span>
</div><div id="l122" class="code_block"><span class="cp">#endif</span>
</div><div id="l123" class="code_block">
</div><div id="l124" class="code_block"><span class="cm">/**</span>
</div><div id="l125" class="code_block"><span class="cm"> * @brief   Enables the SDC subsystem.</span>
</div><div id="l126" class="code_block"><span class="cm"> */</span>
</div><div id="l127" class="code_block"><span class="cp">#if !defined(HAL_USE_SDC) || defined(__DOXYGEN__)</span>
</div><div id="l128" class="code_block"><span class="cp">#define HAL_USE_SDC                 FALSE</span>
</div><div id="l129" class="code_block"><span class="cp">#endif</span>
</div><div id="l130" class="code_block">
</div><div id="l131" class="code_block"><span class="cm">/**</span>
</div><div id="l132" class="code_block"><span class="cm"> * @brief   Enables the SERIAL subsystem.</span>
</div><div id="l133" class="code_block"><span class="cm"> */</span>
</div><div id="l134" class="code_block"><span class="cp">#if !defined(HAL_USE_SERIAL) || defined(__DOXYGEN__)</span>
</div><div id="l135" class="code_block"><span class="cp">#define HAL_USE_SERIAL              TRUE</span>
</div><div id="l136" class="code_block"><span class="cp">#endif</span>
</div><div id="l137" class="code_block">
</div><div id="l138" class="code_block"><span class="cm">/**</span>
</div><div id="l139" class="code_block"><span class="cm"> * @brief   Enables the SERIAL over USB subsystem.</span>
</div><div id="l140" class="code_block"><span class="cm"> */</span>
</div><div id="l141" class="code_block"><span class="cp">#if !defined(HAL_USE_SERIAL_USB) || defined(__DOXYGEN__)</span>
</div><div id="l142" class="code_block"><span class="cp">#define HAL_USE_SERIAL_USB          FALSE</span>
</div><div id="l143" class="code_block"><span class="cp">#endif</span>
</div><div id="l144" class="code_block">
</div><div id="l145" class="code_block"><span class="cm">/**</span>
</div><div id="l146" class="code_block"><span class="cm"> * @brief   Enables the SPI subsystem.</span>
</div><div id="l147" class="code_block"><span class="cm"> */</span>
</div><div id="l148" class="code_block"><span class="cp">#if !defined(HAL_USE_SPI) || defined(__DOXYGEN__)</span>
</div><div id="l149" class="code_block"><span class="cp">#define HAL_USE_SPI                 FALSE</span>
</div><div id="l150" class="code_block"><span class="cp">#endif</span>
</div><div id="l151" class="code_block">
</div><div id="l152" class="code_block"><span class="cm">/**</span>
</div><div id="l153" class="code_block"><span class="cm"> * @brief   Enables the UART subsystem.</span>
</div><div id="l154" class="code_block"><span class="cm"> */</span>
</div><div id="l155" class="code_block"><span class="cp">#if !defined(HAL_USE_UART) || defined(__DOXYGEN__)</span>
</div><div id="l156" class="code_block"><span class="cp">#define HAL_USE_UART                FALSE</span>
</div><div id="l157" class="code_block"><span class="cp">#endif</span>
</div><div id="l158" class="code_block">
</div><div id="l159" class="code_block"><span class="cm">/**</span>
</div><div id="l160" class="code_block"><span class="cm"> * @brief   Enables the USB subsystem.</span>
</div><div id="l161" class="code_block"><span class="cm"> */</span>
</div><div id="l162" class="code_block"><span class="cp">#if !defined(HAL_USE_USB) || defined(__DOXYGEN__)</span>
</div><div id="l163" class="code_block"><span class="cp">#define HAL_USE_USB                 FALSE</span>
</div><div id="l164" class="code_block"><span class="cp">#endif</span>
</div><div id="l165" class="code_block">
</div><div id="l166" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l167" class="code_block"><span class="cm">/* ADC driver related settings.                                              */</span>
</div><div id="l168" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l169" class="code_block">
</div><div id="l170" class="code_block"><span class="cm">/**</span>
</div><div id="l171" class="code_block"><span class="cm"> * @brief   Enables synchronous APIs.</span>
</div><div id="l172" class="code_block"><span class="cm"> * @note    Disabling this option saves both code and data space.</span>
</div><div id="l173" class="code_block"><span class="cm"> */</span>
</div><div id="l174" class="code_block"><span class="cp">#if !defined(ADC_USE_WAIT) || defined(__DOXYGEN__)</span>
</div><div id="l175" class="code_block"><span class="cp">#define ADC_USE_WAIT                TRUE</span>
</div><div id="l176" class="code_block"><span class="cp">#endif</span>
</div><div id="l177" class="code_block">
</div><div id="l178" class="code_block"><span class="cm">/**</span>
</div><div id="l179" class="code_block"><span class="cm"> * @brief   Enables the @p adcAcquireBus() and @p adcReleaseBus() APIs.</span>
</div><div id="l180" class="code_block"><span class="cm"> * @note    Disabling this option saves both code and data space.</span>
</div><div id="l181" class="code_block"><span class="cm"> */</span>
</div><div id="l182" class="code_block"><span class="cp">#if !defined(ADC_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)</span>
</div><div id="l183" class="code_block"><span class="cp">#define ADC_USE_MUTUAL_EXCLUSION    TRUE</span>
</div><div id="l184" class="code_block"><span class="cp">#endif</span>
</div><div id="l185" class="code_block">
</div><div id="l186" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l187" class="code_block"><span class="cm">/* CAN driver related settings.                                              */</span>
</div><div id="l188" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l189" class="code_block">
</div><div id="l190" class="code_block"><span class="cm">/**</span>
</div><div id="l191" class="code_block"><span class="cm"> * @brief   Sleep mode related APIs inclusion switch.</span>
</div><div id="l192" class="code_block"><span class="cm"> */</span>
</div><div id="l193" class="code_block"><span class="cp">#if !defined(CAN_USE_SLEEP_MODE) || defined(__DOXYGEN__)</span>
</div><div id="l194" class="code_block"><span class="cp">#define CAN_USE_SLEEP_MODE          TRUE</span>
</div><div id="l195" class="code_block"><span class="cp">#endif</span>
</div><div id="l196" class="code_block">
</div><div id="l197" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l198" class="code_block"><span class="cm">/* I2C driver related settings.                                              */</span>
</div><div id="l199" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l200" class="code_block">
</div><div id="l201" class="code_block"><span class="cm">/**</span>
</div><div id="l202" class="code_block"><span class="cm"> * @brief   Enables the mutual exclusion APIs on the I2C bus.</span>
</div><div id="l203" class="code_block"><span class="cm"> */</span>
</div><div id="l204" class="code_block"><span class="cp">#if !defined(I2C_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)</span>
</div><div id="l205" class="code_block"><span class="cp">#define I2C_USE_MUTUAL_EXCLUSION    TRUE</span>
</div><div id="l206" class="code_block"><span class="cp">#endif</span>
</div><div id="l207" class="code_block">
</div><div id="l208" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l209" class="code_block"><span class="cm">/* MAC driver related settings.                                              */</span>
</div><div id="l210" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l211" class="code_block">
</div><div id="l212" class="code_block"><span class="cm">/**</span>
</div><div id="l213" class="code_block"><span class="cm"> * @brief   Enables an event sources for incoming packets.</span>
</div><div id="l214" class="code_block"><span class="cm"> */</span>
</div><div id="l215" class="code_block"><span class="cp">#if !defined(MAC_USE_ZERO_COPY) || defined(__DOXYGEN__)</span>
</div><div id="l216" class="code_block"><span class="cp">#define MAC_USE_ZERO_COPY           FALSE</span>
</div><div id="l217" class="code_block"><span class="cp">#endif</span>
</div><div id="l218" class="code_block">
</div><div id="l219" class="code_block"><span class="cm">/**</span>
</div><div id="l220" class="code_block"><span class="cm"> * @brief   Enables an event sources for incoming packets.</span>
</div><div id="l221" class="code_block"><span class="cm"> */</span>
</div><div id="l222" class="code_block"><span class="cp">#if !defined(MAC_USE_EVENTS) || defined(__DOXYGEN__)</span>
</div><div id="l223" class="code_block"><span class="cp">#define MAC_USE_EVENTS              TRUE</span>
</div><div id="l224" class="code_block"><span class="cp">#endif</span>
</div><div id="l225" class="code_block">
</div><div id="l226" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l227" class="code_block"><span class="cm">/* MMC_SPI driver related settings.                                          */</span>
</div><div id="l228" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l229" class="code_block">
</div><div id="l230" class="code_block"><span class="cm">/**</span>
</div><div id="l231" class="code_block"><span class="cm"> * @brief   Delays insertions.</span>
</div><div id="l232" class="code_block"><span class="cm"> * @details If enabled this options inserts delays into the MMC waiting</span>
</div><div id="l233" class="code_block"><span class="cm"> *          routines releasing some extra CPU time for the threads with</span>
</div><div id="l234" class="code_block"><span class="cm"> *          lower priority, this may slow down the driver a bit however.</span>
</div><div id="l235" class="code_block"><span class="cm"> *          This option is recommended also if the SPI driver does not</span>
</div><div id="l236" class="code_block"><span class="cm"> *          use a DMA channel and heavily loads the CPU.</span>
</div><div id="l237" class="code_block"><span class="cm"> */</span>
</div><div id="l238" class="code_block"><span class="cp">#if !defined(MMC_NICE_WAITING) || defined(__DOXYGEN__)</span>
</div><div id="l239" class="code_block"><span class="cp">#define MMC_NICE_WAITING            TRUE</span>
</div><div id="l240" class="code_block"><span class="cp">#endif</span>
</div><div id="l241" class="code_block">
</div><div id="l242" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l243" class="code_block"><span class="cm">/* SDC driver related settings.                                              */</span>
</div><div id="l244" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l245" class="code_block">
</div><div id="l246" class="code_block"><span class="cm">/**</span>
</div><div id="l247" class="code_block"><span class="cm"> * @brief   Number of initialization attempts before rejecting the card.</span>
</div><div id="l248" class="code_block"><span class="cm"> * @note    Attempts are performed at 10mS intervals.</span>
</div><div id="l249" class="code_block"><span class="cm"> */</span>
</div><div id="l250" class="code_block"><span class="cp">#if !defined(SDC_INIT_RETRY) || defined(__DOXYGEN__)</span>
</div><div id="l251" class="code_block"><span class="cp">#define SDC_INIT_RETRY              100</span>
</div><div id="l252" class="code_block"><span class="cp">#endif</span>
</div><div id="l253" class="code_block">
</div><div id="l254" class="code_block"><span class="cm">/**</span>
</div><div id="l255" class="code_block"><span class="cm"> * @brief   Include support for MMC cards.</span>
</div><div id="l256" class="code_block"><span class="cm"> * @note    MMC support is not yet implemented so this option must be kept</span>
</div><div id="l257" class="code_block"><span class="cm"> *          at @p FALSE.</span>
</div><div id="l258" class="code_block"><span class="cm"> */</span>
</div><div id="l259" class="code_block"><span class="cp">#if !defined(SDC_MMC_SUPPORT) || defined(__DOXYGEN__)</span>
</div><div id="l260" class="code_block"><span class="cp">#define SDC_MMC_SUPPORT             FALSE</span>
</div><div id="l261" class="code_block"><span class="cp">#endif</span>
</div><div id="l262" class="code_block">
</div><div id="l263" class="code_block"><span class="cm">/**</span>
</div><div id="l264" class="code_block"><span class="cm"> * @brief   Delays insertions.</span>
</div><div id="l265" class="code_block"><span class="cm"> * @details If enabled this options inserts delays into the MMC waiting</span>
</div><div id="l266" class="code_block"><span class="cm"> *          routines releasing some extra CPU time for the threads with</span>
</div><div id="l267" class="code_block"><span class="cm"> *          lower priority, this may slow down the driver a bit however.</span>
</div><div id="l268" class="code_block"><span class="cm"> */</span>
</div><div id="l269" class="code_block"><span class="cp">#if !defined(SDC_NICE_WAITING) || defined(__DOXYGEN__)</span>
</div><div id="l270" class="code_block"><span class="cp">#define SDC_NICE_WAITING            TRUE</span>
</div><div id="l271" class="code_block"><span class="cp">#endif</span>
</div><div id="l272" class="code_block">
</div><div id="l273" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l274" class="code_block"><span class="cm">/* SERIAL driver related settings.                                           */</span>
</div><div id="l275" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l276" class="code_block">
</div><div id="l277" class="code_block"><span class="cm">/**</span>
</div><div id="l278" class="code_block"><span class="cm"> * @brief   Default bit rate.</span>
</div><div id="l279" class="code_block"><span class="cm"> * @details Configuration parameter, this is the baud rate selected for the</span>
</div><div id="l280" class="code_block"><span class="cm"> *          default configuration.</span>
</div><div id="l281" class="code_block"><span class="cm"> */</span>
</div><div id="l282" class="code_block"><span class="cp">#if !defined(SERIAL_DEFAULT_BITRATE) || defined(__DOXYGEN__)</span>
</div><div id="l283" class="code_block"><span class="cp">#define SERIAL_DEFAULT_BITRATE      38400</span>
</div><div id="l284" class="code_block"><span class="cp">#endif</span>
</div><div id="l285" class="code_block">
</div><div id="l286" class="code_block"><span class="cm">/**</span>
</div><div id="l287" class="code_block"><span class="cm"> * @brief   Serial buffers size.</span>
</div><div id="l288" class="code_block"><span class="cm"> * @details Configuration parameter, you can change the depth of the queue</span>
</div><div id="l289" class="code_block"><span class="cm"> *          buffers depending on the requirements of your application.</span>
</div><div id="l290" class="code_block"><span class="cm"> * @note    The default is 64 bytes for both the transmission and receive</span>
</div><div id="l291" class="code_block"><span class="cm"> *          buffers.</span>
</div><div id="l292" class="code_block"><span class="cm"> */</span>
</div><div id="l293" class="code_block"><span class="cp">#if !defined(SERIAL_BUFFERS_SIZE) || defined(__DOXYGEN__)</span>
</div><div id="l294" class="code_block"><span class="cp">#define SERIAL_BUFFERS_SIZE         16</span>
</div><div id="l295" class="code_block"><span class="cp">#endif</span>
</div><div id="l296" class="code_block">
</div><div id="l297" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l298" class="code_block"><span class="cm">/* SPI driver related settings.                                              */</span>
</div><div id="l299" class="code_block"><span class="cm">/*===========================================================================*/</span>
</div><div id="l300" class="code_block">
</div><div id="l301" class="code_block"><span class="cm">/**</span>
</div><div id="l302" class="code_block"><span class="cm"> * @brief   Enables synchronous APIs.</span>
</div><div id="l303" class="code_block"><span class="cm"> * @note    Disabling this option saves both code and data space.</span>
</div><div id="l304" class="code_block"><span class="cm"> */</span>
</div><div id="l305" class="code_block"><span class="cp">#if !defined(SPI_USE_WAIT) || defined(__DOXYGEN__)</span>
</div><div id="l306" class="code_block"><span class="cp">#define SPI_USE_WAIT                TRUE</span>
</div><div id="l307" class="code_block"><span class="cp">#endif</span>
</div><div id="l308" class="code_block">
</div><div id="l309" class="code_block"><span class="cm">/**</span>
</div><div id="l310" class="code_block"><span class="cm"> * @brief   Enables the @p spiAcquireBus() and @p spiReleaseBus() APIs.</span>
</div><div id="l311" class="code_block"><span class="cm"> * @note    Disabling this option saves both code and data space.</span>
</div><div id="l312" class="code_block"><span class="cm"> */</span>
</div><div id="l313" class="code_block"><span class="cp">#if !defined(SPI_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)</span>
</div><div id="l314" class="code_block"><span class="cp">#define SPI_USE_MUTUAL_EXCLUSION    TRUE</span>
</div><div id="l315" class="code_block"><span class="cp">#endif</span>
</div><div id="l316" class="code_block">
</div><div id="l317" class="code_block"><span class="cp">#endif </span><span class="cm">/* _HALCONF_H_ */</span><span class="cp"></span>
</div><div id="l318" class="code_block">
</div><div id="l319" class="code_block"><span class="cm">/** @} */</span>
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