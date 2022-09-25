package com.example.taxi.data.repository

import android.util.Log
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.calltaxi.TaxiList
import com.example.taxi.data.dto.user.destination.FrequentDestination
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
        database.collection(FireStoreCollection.ROUTE).document("Route")
            .get()
            .addOnSuccessListener { document ->
                Log.d("getRoute", "DocumentSnapshot data: ${document.data}")
                if (document != null) {
                    val route = document.toObject(Route::class.java)
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
            .addOnFailureListener {
                Log.d("getRoute", "Fail get document")
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
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

    override fun getTaxiList(result: (UiState<List<Taxi>>) -> Unit) {
        database.collection(FireStoreCollection.TAXILIST).document("Alpaca")
            .get()
            .addOnSuccessListener { document ->
                Log.d("getTaxiList", "DocumentSnapshot data: ${document.data}")
                if (document != null) {
                    val taxiList = document.toObject(TaxiList::class.java)
                    if(taxiList != null){
                        val taxiList = taxiList.taxiList
                        val newTaxiList = mutableListOf<Taxi>()
                        if(taxiList != null){
                            for(des in taxiList){
                                val taxi = Taxi(carImage = des.carImage, carNumber = des.carNumber,
                                isEachDriving = des.isEachDriving, isEachInOperation = des.isEachInOperation,
                                rideComfortAverage = des.rideComfortAverage, cleanlinessAverage = des.cleanlinessAverage,
                                position = des.position)
                                newTaxiList.add(taxi)
                            }
                            result.invoke(
                                UiState.Success(newTaxiList)
                            )
                        }
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
}