function time()
{
    var d1 = new Date();
    var time = "";
    time = d1.toTimeString().split(' ')[0];
    time += "  ";
    time += d1.toLocaleDateString();
     
    var timeDiv = document.getElementById("time");
    timeDiv.innerText = time;
}