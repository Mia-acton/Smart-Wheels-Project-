function thankYou(){
    var fname=document.getElementById("fname").value;
    var phone=document.getElementById("phone").value;
    var email=document.getElementById("email").value;

    window.alert('Thank you ' + fname + ' for signing in. Someone will call ('+phone+') or email ('+email+') shortly with an authentication key. We appreciate your patience!')
    location.reload();
}