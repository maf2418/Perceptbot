

// (function ($) {

//     'use strict';

//     $(document).ready(function () {

//         // Init here.
//         var $body = $('body'),
//             $main = $('#main'),
//             $site = $('html, body'),
//             transition = 'fade',
//             smoothState;

//         smoothState = $main.smoothState({
//             onBefore: function($anchor, $container) {
//                 var current = $('[data-viewport]').first().data('viewport'),
//                     target = $anchor.data('target');
//                 current = current ? current : 0;
//                 target = target ? target : 0;
//                 if (current === target) {
//                     transition = 'fade';
//                 } else if (current < target) {
//                     transition = 'moveright';
//                 } else {
//                     transition = 'moveleft';
//                 }
//             },
//             onStart: {
//                 duration: 400,
//                 render: function (url, $container) {
//                     $main.attr('data-transition', transition);
//                     $main.addClass('is-exiting');
//                     $site.animate({scrollTop: 0});
//                 }
//             },
//             onReady: {
//                 duration: 0,
//                 render: function ($container, $newContent) {
//                     $container.html($newContent);
//                     $container.removeClass('is-exiting');
//                 }
//             },
//         }).data('smoothState');

//     });

// }(jQuery));

// (function ($) {
//   'use strict'

//   $(document).ready(function () {
//     // Init here.

//     var casd_init = function(casd) {

//         if(typeof casd == 'undefined') {
//             //alert("casd undefined");
//         }
//         else {
//             //alert("cas set");
//             for(let index = 0; index < casd.length; ++index)
//             {
//             if( casd[index].innerHTML == undefined ) { next; }
//             casd[index].iblock1hidden = false;   // has not been declared ..
//             casd[index].innerHTML = set_minus(casd[index].innerHTML);
//             }
//         }
//     }

//     var casd_click = function() {
//         //alert('casd clicked');
//         if(this.iblock1hidden) {
//             $("div#"+this.id).show(1000);
//             this.iblock1hidden = false;
//             this.innerHTML = swap_plus_minus(this.innerHTML);
//         }
//         else {
//             $("div#"+this.id).hide(1000);
//             this.iblock1hidden = true;
//             this.innerHTML = swap_plus_minus(this.innerHTML);
//         }
//     }   // casd_click

//     var casd = $('.casd');
//     casd_init(casd);
//     casd.click(casd_click);   // casd.click

//     console.log('document.ready is initializing smoothState');
//     var $body = $('body'),
//     $main = $('#main'),
//     $site = $('html, body'),
//     transition = 'fade',
//     smoothState;

//     smoothState = $main
//       .smoothState({
//         onBefore: ($anchor, $container) => {
//             console.log('smoothState onBefore');
//             var current = $('[data-viewport]')
//             .first()
//             .data('viewport');

//           var target = $anchor.data('target');
//           current = current || 0;
//           target = target || 0;
//           if (current === target) {
//             transition = 'fade';
//           } else if (current < target) {
//             transition = 'moveright';
//           } else {
//             transition = 'moveleft';
//           }
//         },
//         onStart: {
//           duration: 400,
//           render: (url, $container) => {
//             console.log('smoothState onStart render');
//             $main.attr('data-transition', transition);
//             $main.addClass('is-exiting');
//             $site.animate({ scrollTop: 0 });
//           }
//         },
//         onReady: {
//           duration: 0,
//           render: ($container, $newContent) => {
//             console.log('smoothState onReady render');
//             $container.html($newContent);
//             $container.removeClass('is-exiting');
//           }
//         },

//         // hook-up the functionality from function.js
//         onAfter: ($container, $newContent) => {
//             console.log('smoothState onAfter');
//           var casd = $('.casd');
//           casd_init(casd);
//           casd.click(casd_click); // casd.click
//         } // onAfter
//       })
//       .data('smoothState')

//     // var sliderB = new Slider("#ex17b", {
//     //     min: 0,
//     //     max: 10,
//     //     value: 0,
//     //     orientation: 'vertical',
//     //     tooltip_position:'left'
//     // });
//   });
// })(jQuery);
