package com.example.taxi.data.repository

import android.util.Log
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.calltaxi.TaxiList
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.data.dto.user.driving.CurrentLocation
import com.example.taxi.data.dto.user.route.Distance
import com.example.taxi.data.dto.user.route.Location
import com.example.taxi.data.dto.user.route.Route
import com.example.taxi.data.dto.user.route.RouteSetting
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.FireStoreCollection
import com.example.taxi.utils.constant.UiState
import com.google.firebase.firestore.FirebaseFirestore

class RouteRepositoryImpl(
    private val database: FirebaseFirestore
) : RouteRepository {
    override fun getRoute(result: (UiState<List<String>>) -> Unit) {
        database.collection(FireStoreCollection.ROUTE).document("12가3456")
            .addSnapshotListener { snapshot, e ->
                if (e != null) {
                    Log.w("getRoute", "listen:error", e)
                    result.invoke(
                        UiState.Failure(
                            e.localizedMessage
                        )
                    )
                    return@addSnapshotListener
                }
                if (snapshot != null) {
                    Log.d("getRoute", "DocumentSnapshot data: ${snapshot.data}")
                    val route = snapshot.toObject(Route::class.java)
                    if(route != null){
                        val locations = route.route
                        result.invoke(
                            UiState.Success(locations)
                        )
                    }
                } else {
                    Log.d("getRoute", "No such document")
                }
            }
    }

    override fun getDistance(result: (UiState<Int>) -> Unit) {
        database.collection(FireStoreCollection.DISTANCE).document("12가3456")
            .addSnapshotListener { snapshot, e ->
                if (e != null) {
                    Log.w("getCurrentLocation", "listen:error", e)
                    result.invoke(
                        UiState.Failure(
                            e.localizedMessage
                        )
                    )
                    return@addSnapshotListener
                }

                if (snapshot != null) {
                    Log.d("getDistance", "DocumentSnapshot data: ${snapshot.data}")
                    val distance = snapshot.toObject(Distance::class.java)
                    if(distance != null){
                        result.invoke(
                            UiState.Success(distance.dis)
                        )
                    }
                } else {
                    Log.d("getDistance", "No such document")
                }
            }
    }

    override fun addRouteSetting(
        routeSetting: RouteSetting,
        result: (UiState<RouteSetting>) -> Unit
    ) {
        val document = database.collection(FireStoreCollection.ROUTESETTING).document("12가3456")
        document
            .set(routeSetting)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(routeSetting)
                )
                Log.d("addRouteSetting", "Destination has been created successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("addRouteSetting", "Destination has been created fail")
            }
    }

    override fun getTaxiList(result: (UiState<Taxi>) -> Unit) {
        database.collection(FireStoreCollection.TAXILIST).document("Alpaca")
            .get()
            .addOnSuccessListener { document ->
                Log.d("getTaxiList", "DocumentSnapshot data: ${document.data}")
                if (document != null) {
                    val taxiList = document.toObject(Taxi::class.java)
                    if(taxiList != null){
                        result.invoke(
                            UiState.Success(taxiList)
                        )
                    }
                } else {
                    Log.d("getTaxiList", "No such document")
                }
            }
            .addOnFailureListener {
                Log.d("getTaxiList", "Fail get document")
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }
    }

    override fun updateTaxiList(
        taxiList: Taxi,
        result: (UiState<Taxi>) -> Unit
    ) {
        val document = database.collection(FireStoreCollection.TAXILIST).document("Alpaca")
        document
            .update("isEachInOperation",taxiList.isEachInOperation)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(taxiList)
                )
                Log.d("updateTaxiList", "Destination has been created successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("updateTaxiList", "Destination has been created fail")
            }
    }

    override fun getCurrentLocation(result: (UiState<CurrentLocation>) -> Unit) {
        database.collection(FireStoreCollection.CURRENTLOCATION).document("Ego_0")
            .addSnapshotListener { snapshot, e ->
                if (e != null) {
                    Log.w("getCurrentLocation", "listen:error", e)
                    result.invoke(
                        UiState.Failure(
                            e.localizedMessage
                        )
                    )
                    return@addSnapshotListener
                }
                if (snapshot != null) {
                    Log.d("getCurrentLocation", "DocumentSnapshot data: ${snapshot.data}")
                    val currentLocation = snapshot.toObject(CurrentLocation::class.java)
                    if(currentLocation != null){
                        result.invoke(
                            UiState.Success(currentLocation)
                        )
                    }
                } else {
                    Log.d("getCurrentLocation", "No such document")
                }
            }
    }

}