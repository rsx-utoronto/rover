var mymap = L.map('mapid').setView([43.6532, -79.3832], 13);

L.tileLayer('https://api.tiles.mapbox.com/v4/mapbox.streets/{z}/{x}/{y}.png?access_token=pk.eyJ1IjoiZWxsZW54d2NoZW4iLCJhIjoiY2szbHZlc2RiMDJiejNmcWVteGx3aHNjcSJ9.KTf1PHkOPk_pXswRLLu8YA', {
    attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors, <a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
    maxZoom: 18,
}).addTo(mymap);

mymap.on('click', addMarker);

function addMarker(e){
  
  //add marker
  circleMarker = new  L.circle(e.latlng, 5, {
                color: 'red',
                fillColor: '#f03',
                fillOpacity: 1.0
            }).addTo(mymap);
}
