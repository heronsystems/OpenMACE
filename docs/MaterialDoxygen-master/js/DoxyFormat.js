//==--- MaterialDoxygen/Js/DoxyFormat.js -------------------- -*- Js -*- ---==//
//
//                           Voxel : MaterialDoxygen
//
//                        Copyright (c) 2017 Rob Clucas
//
//  This file is distributed under the MIT License. See LICENSE for details. 
//
//==------------------------------------------------------------------------==//
//
/// File          : DoxyFormat.js
/// Description   : Provides functionality for formatting doxygen output to use
//                  a voxel-modified version of materialize css when follows
//                  the formatting of the rest of the internal documentation.
//
//==-----------------------------------------------------------------------==//

var mainHeading = "h2";
var subHeading  = "h3";

$(document).ready(function(){
  $('.button-collapse').sideNav();
  $('.scrollspy').scrollSpy();
  $('.collapsible').collapsible();

  // Support for the menu:
  $('.button-collapse').on('click', function(){
    if ($(this).hasClass('active')) {
      $(this).removeClass('active');
      $('.site-content').removeClass('shift-right');
    } else {
      $(this).addClass('active');
      $('.site-content').addClass('shift-right');
    }
  });

  $('body').on('click', function() {
    if ($('.button-collapse').hasClass('active')) {
      $('.button-collapse').removeClass('active');
      $('.site-content').removeClass('shift-right');
    }
  });

  //==---  Modifications to doxygen: --------------------------------------==//
  
  $('table').addClass('highlighted').removeClass('doxtable');
  $('table.directory').removeClass('directory');

  $('div.header').remove();
  var navRow1  = document.getElementById('navrow1'); 
  var navRow2  = document.getElementById('navrow2'); 
  var navRow3  = document.getElementById('navrow3'); 
  var navRow4  = document.getElementById('navrow4'); 
  var mainNav  = document.getElementById('main-nav');
  var navPath  = document.getElementById('nav-path');
  if (navRow1 != null) { navRow1.outerHTML = ""; }
  if (navRow2 != null) { navRow2.outerHTML = ""; }
  if (navRow3 != null) { navRow3.outerHTML = ""; }
  if (navRow4 != null) { navRow4.outerHTML = ""; }
  if (mainNav != null) { mainNav.outerHTML = ""; }
  if (navPath != null) { navPath.outerHTML = ""; }

  var currentUrl = window.location.href.split('/').pop();

  // Remove doxygen search:
  document.getElementById('MSearchResultsWindow').outerHTML = "";
  document.getElementById('MSearchSelectWindow').outerHTML  = "";

  // Remove levels:
  $('.levels').remove();

  // Remove doxygen text block and add our own contents:
  $('.textblock').removeClass('textblock');
  $('.contents').addClass("col s12 m8 l6");
  $('.contents').wrap("<div class='row'></div>");
  $('.contents').before("<div class='col s0 m2 l3'></div>");
  $('.contents').prepend(
    "<div class='row'>"                                                        +
      "<div class='col s12'>"                                                  +
        "<ul class='tabs z-depth-1'>"                                          +
          "<li class='tab col s3'><a href='index.html'>Home</a></li>"          +
          "<li class='tab col s3'><a href='annotated.html'>Namespaces</a></li>"+
          "<li class='tab col s3'><a href='classes.html'>Classes</a></li>"     +
          "<li class='tab col s3'><a href='files.html'>Files</a></li>"         +
        "</ul>"                                                                +
      "</div>"                                                                 +
    "</div>");
  $('.contents').after("<div id='tabofcontents' class='col s0 m2 l3'></div>");

  //==--- Code modifications -----------------------------------------------==//

  $('.fragment').wrap("<code></code>");
  $(".fragment").addClass('highlight');
  $('.fragment').removeClass('fragment');

  $('.comment').addClass('c1').removeClass('comment');
  $('.keyword').addClass('kt').removeClass('keyword');
  $('.keywordflow').addClass('kr').removeClass('keywordflow');
  $('.preprocessor').addClass('cp').removeClass('preprocessor');
  $('.stringliteral').addClass('mi').removeClass('stringliteral');
  $('.keywordtype').addClass('nf').removeClass('keywordtype');
  $('a.code').addClass('k').removeClass('code');
  $('a.line').addClass('newline').removeClass('line');
  $('.ttc').remove();
  $('div.line').replaceWith(function() {
    return $("<span class='line'>" + this.innerHTML + "</span>");
  });
 
  //==--- Files Modifications ----------------------------------------------==//

  $('a.el').addClass('tablelink');
  $('td.entry').addClass('voxeltd');
  $('td.entry').removeClass('entry');

  // Modifications for the file link:
  if (currentUrl == "files.html") {
    $('.iconfopen').addClass('material-icons');
    $('.iconfopen').addClass('iconlink');
    $('.iconfopen').text("folder");
    $('.iconfopen').removeClass("iconfopen");

    $('.icondoc').addClass('material-icons');
    $('.icondoc').addClass('iconlink');
    $('.icondoc').text('description');
    $('.icondoc').removeClass('icondoc');
  }

  //==--- Project structure Modifications ----------------------------------==//

  if (currentUrl == "annotated.html") {
    $('.icona').addClass('material-icons'); 
    $('.icona').addClass('iconlink'); 
    $('.icona').text('menu');
    $('.icona').removeClass("icona");
    $('b').addClass('tablelink');
  }

  //==--- Detail Modifications ---------------------------------------------==//

  var namespaceDescriptionFile = currentUrl.search("namespace");
  if (currentUrl == "namespacemembers.html")
    namespaceDescriptionFile = -1;

  if (currentUrl.search("struct")     != -1 || 
      currentUrl.search("class")      != -1 ||
      namespaceDescriptionFile        != -1 ||
      currentUrl.search("hpp")        != -1 ||
      currentUrl.search("cpp")        != -1) {
   
    $('table.memname.highlighted').removeClass('highlighted');
    $('table.mlabels.highlighted').removeClass('highlighted');

    $('.table').each(function() {
      var table = $(this);
      var removable = table.hasClass('memname') || table.hasClass('mlabels');
      if (removable && table.hasClass('highlighted')) {
        table.removeClass('highlighted');
      }
    });

    var header   = $('h3');
    var elements = header.nextUntil('h2');
    header.addClass('doxyH3');

    if (header.length > 0) {
      // Make a card:
      var newCard = 
          "<div class='card z-depth-3'>"                                   +
            "<div class='card-content'>"                                   +
                "<span class='bold-card-title card-title'>"                +
                  header[0].innerHTML                                      +
                "</span>"                                                  +
            "</div>"                                                       +
            "<div class='card-action'>";

      // For each of the elements, add them to the card action:
      for (var i = 0 ; i < elements.length; i++) {
        newCard += elements[i].outerHTML;
      }
      newCard += "</div></div>";
      header[0].parentElement.outerHTML = newCard;
    }

    $('a.el').addClass('tablelink');

    var item, memproto;
    $('.memitem').each(function() {
      item = $(this);
      item.addClass('card').addClass('.z-depth-1');
      
      item.children('.memproto').each(function() {
        $(this).addClass('bold-card-title').addClass('card-title');
        $(this).removeClass('memproto');
      });
      item.removeClass('memitem');
    });

    $('.memdoc').each(function() {
      item = $(this);
      item.addClass('card-action');
      item.removeClass('memdoc');
    });

    $('.memtitle').each(function() {
      (this).remove();
    });
  }

  //== Enumerations -------------------------------------------------------==//
  
  $('.fieldtable').each(function() {
    var elem = $(this);
    if (elem.hasClass('highlighted')) {
      elem.removeClass('highlighted');
    }
    elem.addClass('striped');
    elem.removeClass('fieldtable');
  });
});

