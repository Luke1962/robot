<!DOCTYPE html>
<!-- Server: sfn-web-13 -->


    















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
  /trunk_old/demos/AVR-Arduino-GCC/mcuconf.h
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

    <style>.XkVIHCrduobXfsODybIqYNqTA {
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
    
  
 mcuconf.h

                    <!-- actions -->
                    <small>
                        

    
    <a class="icon" href="#" id="maximize-content" title="Maximize"><i class="fa fa-expand"></i>&nbsp;Maximize</a>
    <a class="icon" href="#" id="restore-content" title="Restore"><i class="fa fa-compress"></i>&nbsp;Restore</a>
<a class="icon" href="/p/chibios/svn/8742/log/?path=/trunk_old/demos/AVR-Arduino-GCC/mcuconf.h" title="History"><i class="fa fa-calendar"></i>&nbsp;History</a>

                    </small>
                    <!-- /actions -->
                </h2>
                
                <div>
                    
  

                    
  
    <p><a href="?format=raw">Download this file</a></p>
    <div class="clip grid-19 codebrowser">
      <h3>
        80 lines (66 with data), 2.3 kB
      </h3>
      
        <table class="codehilitetable"><tr><td class="linenos"><div class="linenodiv"><pre> 1
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
79</pre></div></td><td class="code"><div class="codehilite"><pre><div id="l1" class="code_block"><span class="cm">/*</span>
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
</div><div id="l17" class="code_block"><span class="cm">/*</span>
</div><div id="l18" class="code_block"><span class="cm"> * AVR drivers configuration.</span>
</div><div id="l19" class="code_block"><span class="cm"> * The following settings override the default settings present in</span>
</div><div id="l20" class="code_block"><span class="cm"> * the various device driver implementation headers.</span>
</div><div id="l21" class="code_block"><span class="cm"> * Note that the settings for each driver only have effect if the driver</span>
</div><div id="l22" class="code_block"><span class="cm"> * is enabled in halconf.h.</span>
</div><div id="l23" class="code_block"><span class="cm"> */</span>
</div><div id="l24" class="code_block">
</div><div id="l25" class="code_block"><span class="cm">/*</span>
</div><div id="l26" class="code_block"><span class="cm"> * ADC driver system settings.</span>
</div><div id="l27" class="code_block"><span class="cm"> */</span>
</div><div id="l28" class="code_block"><span class="cp">#define AVR_ADC_USE_ADC1                   FALSE</span>
</div><div id="l29" class="code_block">
</div><div id="l30" class="code_block"><span class="cm">/*</span>
</div><div id="l31" class="code_block"><span class="cm"> * CAN driver system settings.</span>
</div><div id="l32" class="code_block"><span class="cm"> */</span>
</div><div id="l33" class="code_block">
</div><div id="l34" class="code_block"><span class="cm">/*</span>
</div><div id="l35" class="code_block"><span class="cm"> * MAC driver system settings.</span>
</div><div id="l36" class="code_block"><span class="cm"> */</span>
</div><div id="l37" class="code_block">
</div><div id="l38" class="code_block"><span class="cm">/*</span>
</div><div id="l39" class="code_block"><span class="cm"> * PWM driver system settings.</span>
</div><div id="l40" class="code_block"><span class="cm"> */</span>
</div><div id="l41" class="code_block"><span class="cp">#define AVR_PWM_USE_TIM1                   FALSE</span>
</div><div id="l42" class="code_block"><span class="cp">#define AVR_PWM_USE_TIM2                   FALSE</span>
</div><div id="l43" class="code_block"><span class="cp">#define AVR_PWM_USE_TIM3                   FALSE</span>
</div><div id="l44" class="code_block"><span class="cp">#define AVR_PWM_USE_TIM4                   FALSE</span>
</div><div id="l45" class="code_block"><span class="cp">#define AVR_PWM_USE_TIM5                   FALSE</span>
</div><div id="l46" class="code_block">
</div><div id="l47" class="code_block"><span class="cm">/*</span>
</div><div id="l48" class="code_block"><span class="cm"> * ICU driver system settings.</span>
</div><div id="l49" class="code_block"><span class="cm"> */</span>
</div><div id="l50" class="code_block"><span class="cp">#define AVR_ICU_USE_TIM1                   FALSE</span>
</div><div id="l51" class="code_block"><span class="cp">#define AVR_ICU_USE_TIM3                   FALSE</span>
</div><div id="l52" class="code_block"><span class="cp">#define AVR_ICU_USE_TIM4                   FALSE</span>
</div><div id="l53" class="code_block"><span class="cp">#define AVR_ICU_USE_TIM5                   FALSE</span>
</div><div id="l54" class="code_block">
</div><div id="l55" class="code_block"><span class="cm">/*</span>
</div><div id="l56" class="code_block"><span class="cm"> * GPT driver system settings.</span>
</div><div id="l57" class="code_block"><span class="cm"> */</span>
</div><div id="l58" class="code_block"><span class="cp">#define AVR_GPT_USE_TIM1                   FALSE</span>
</div><div id="l59" class="code_block"><span class="cp">#define AVR_GPT_USE_TIM2                   FALSE</span>
</div><div id="l60" class="code_block"><span class="cp">#define AVR_GPT_USE_TIM3                   FALSE</span>
</div><div id="l61" class="code_block"><span class="cp">#define AVR_GPT_USE_TIM4                   FALSE</span>
</div><div id="l62" class="code_block"><span class="cp">#define AVR_GPT_USE_TIM5                   FALSE</span>
</div><div id="l63" class="code_block">
</div><div id="l64" class="code_block"><span class="cm">/*</span>
</div><div id="l65" class="code_block"><span class="cm"> * SERIAL driver system settings.</span>
</div><div id="l66" class="code_block"><span class="cm"> */</span>
</div><div id="l67" class="code_block"><span class="cp">#define AVR_SERIAL_USE_USART0              TRUE</span>
</div><div id="l68" class="code_block"><span class="cp">#define AVR_SERIAL_USE_USART1              FALSE</span>
</div><div id="l69" class="code_block">
</div><div id="l70" class="code_block"><span class="cm">/*</span>
</div><div id="l71" class="code_block"><span class="cm"> * I2C driver system settings.</span>
</div><div id="l72" class="code_block"><span class="cm"> */</span>
</div><div id="l73" class="code_block"><span class="cp">#define AVR_I2C_USE_I2C1                   FALSE</span>
</div><div id="l74" class="code_block">
</div><div id="l75" class="code_block"><span class="cm">/*</span>
</div><div id="l76" class="code_block"><span class="cm"> * SPI driver system settings.</span>
</div><div id="l77" class="code_block"><span class="cm"> */</span>
</div><div id="l78" class="code_block"><span class="cp">#define AVR_SPI_USE_SPI1                   FALSE</span>
</div><div id="l79" class="code_block"><span class="cp">#define AVR_SPI_USE_16BIT_POLLED_EXCHANGE  FALSE</span>
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