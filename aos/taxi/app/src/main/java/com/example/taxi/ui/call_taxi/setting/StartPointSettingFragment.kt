package com.example.taxi.ui.call_taxi.setting

import android.content.pm.PackageManager
import android.location.Geocoder
import android.util.Log
import androidx.annotation.UiThread
import androidx.core.app.ActivityCompat
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentStartPointSettingBinding
import com.naver.maps.map.CameraUpdate
import com.naver.maps.map.MapFragment
import com.naver.maps.map.NaverMap
import com.naver.maps.map.OnMapReadyCallback
import com.naver.maps.map.overlay.Marker
import com.naver.maps.map.overlay.OverlayImage
import com.naver.maps.map.util.FusedLocationSource
import dagger.hilt.android.AndroidEntryPoint
import java.io.IOException
import java.util.*
import kotlin.collections.ArrayList
import android.Manifest
import android.graphics.Color
import android.location.Address
import android.location.Location
import android.widget.Toast
import androidx.core.os.bundleOf
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.utils.view.toast
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationServices
import com.gun0912.tedpermission.PermissionListener
import com.gun0912.tedpermission.normal.TedPermission
import com.naver.maps.geometry.LatLng
import com.ssafy.daero.utils.view.getPxFromDp

@AndroidEntryPoint
class StartPointSettingFragment : BaseFragment<FragmentStartPointSettingBinding>(R.layout.fragment_start_point_setting),
    OnMapReadyCallback {
    private lateinit var fusedLocationClient: FusedLocationProviderClient
    private lateinit var locationSource: FusedLocationSource
    private lateinit var naverMap: NaverMap
    private var destination = Destination("","","","")
    private lateinit var startingPoint : Destination
    var checkState = false
    var checkCamera = false

    override fun init() {
        fusedLocationClient = LocationServices.getFusedLocationProviderClient(requireContext())
        requestPermissions()
        initData()
        initView()
        setOnClickListeners()
    }

    private fun initData() {
        if(arguments?.getParcelable<Destination>("Destination")!=null){
            destination = arguments?.getParcelable<Destination>("Destination") as Destination
            binding.textStartingPointSettingEndAddress.text = destination.addressName
        }
    }

    private fun initView() {
        val _naverMap =
            childFragmentManager.findFragmentById(R.id.fragmentContainer_starting_point_setting) as MapFragment?
                ?: MapFragment.newInstance().also {
                    childFragmentManager.beginTransaction()
                        .add(R.id.fragmentContainer_starting_point_setting, it)
                        .commit()
                }
        _naverMap.getMapAsync(this)
    }

    // 위치권한 관련 요청
    private fun requestPermissions() {
        // 내장 위치 추적 기능 사용
        locationSource =
            FusedLocationSource(this, LOCATION_PERMISSION_REQUEST_CODE)
        TedPermission.create()
            .setRationaleTitle("위치권한 요청")
            .setPermissionListener(object : PermissionListener {
                override fun onPermissionGranted() {
                    toast("위치 정보 제공이 완료되었습니다.")
                }

                override fun onPermissionDenied(deniedPermissions: MutableList<String>?) {
                    toast("위치 정보 제공이 거절되었습니다.")
                }

            })
            .setRationaleMessage("현재 위치로 이동하기 위해 위치권한이 필요합니다.") // "we need permission for read contact and find your location"
            .setDeniedMessage("[설정] -> [권한]에서 권한 변경이 가능합니다.")
            .setDeniedCloseButtonText("닫기")
            .setGotoSettingButtonText("설정")
            .setPermissions(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
            .check()


    }

    private fun setOnClickListeners() {
        binding.buttonStartDrivingTaxiStart.setOnClickListener {
            startingPoint = Destination(binding.textStartingPointSettingAddress.text.toString(),
                naverMap.cameraPosition.target.latitude.toString(),binding.textStartingPointSettingAddress.text.toString()
                ,naverMap.cameraPosition.target.longitude.toString()
            )
            if(destination.address==""){
                findNavController().navigate(R.id.action_startPointSettingFragment_to_destinationSettingFragment,
                    bundleOf("StartingPoint" to startingPoint)
                )
            }else{
                findNavController().navigate(R.id.action_startPointSettingFragment_to_callTaxiFragment,
                    bundleOf("Destination" to destination, "StartingPoint" to startingPoint)
                )
            }
        }
    }

    // 네이버맵 불러오기가 완료되면 콜백
    @UiThread
    override fun onMapReady(_naverMap: NaverMap) {
        this.naverMap = _naverMap
        // 내장 위치 추적 기능 사용
        naverMap.locationSource = locationSource

        // 빨간색 표시 마커 (네이버맵 현재 가운데에 항상 위치)
        val marker = Marker()
        marker.position = LatLng(
            naverMap.cameraPosition.target.latitude,
            naverMap.cameraPosition.target.longitude
        )
        marker.icon = OverlayImage.fromResource(R.drawable.ic_marker)
        marker.iconTintColor = requireActivity().getColor(R.color.primaryColor)// 마커 색깔
        marker.width = requireContext().getPxFromDp(40f)   // 마커 가로 크기
        marker.height = requireContext().getPxFromDp(40f)  // 마커 세로 크기
        marker.map = naverMap

        // 카메라의 움직임에 대한 이벤트 리스너 인터페이스.
        // 참고 : https://navermaps.github.io/android-map-sdk/reference/com/naver/maps/map/package-summary.html
        naverMap.addOnCameraChangeListener { reason, animated ->
            Log.i("NaverMap", "카메라 변경 - reson: $reason, animated: $animated")
            marker.position = LatLng(
                // 현재 보이는 네이버맵의 정중앙 가운데로 마커 이동
                naverMap.cameraPosition.target.latitude,
                naverMap.cameraPosition.target.longitude
            )
            // 주소 텍스트 세팅 및 확인 버튼 비활성화
            binding.textStartingPointSettingAddress.run {
                text = "위치 이동 중"
                setTextColor(Color.parseColor("#c4c4c4"))
            }
        }

        // 카메라의 움직임 종료에 대한 이벤트 리스너 인터페이스.
        naverMap.addOnCameraIdleListener {
            marker.position = LatLng(
                naverMap.cameraPosition.target.latitude,
                naverMap.cameraPosition.target.longitude
            )
            // 좌표 -> 주소 변환 텍스트 세팅, 버튼 활성화
            binding.textStartingPointSettingAddress.run {
                text = getAddress(
                    naverMap.cameraPosition.target.latitude,
                    naverMap.cameraPosition.target.longitude
                )
                setTextColor(Color.parseColor("#2d2d2d"))
            }
        }

        if (ActivityCompat.checkSelfPermission(
                requireContext(),
                Manifest.permission.ACCESS_FINE_LOCATION
            ) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(
                requireContext(),
                Manifest.permission.ACCESS_COARSE_LOCATION
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            return
        }
        // 사용자 현재 위치 받아오기
        var currentLocation: Location?
        fusedLocationClient.lastLocation
            .addOnSuccessListener { location: Location? ->
                currentLocation = location
                // 위치 오버레이의 가시성은 기본적으로 false로 지정되어 있습니다. 가시성을 true로 변경하면 지도에 위치 오버레이가 나타납니다.
                // 파랑색 점, 현재 위치 표시
                naverMap.locationOverlay.run {
                    isVisible = true
                    position = if(currentLocation != null && checkState){
                        LatLng(currentLocation!!.latitude, currentLocation!!.longitude)
                    }else{
                        checkState = true
                        LatLng(37.58253888738363,126.88582972882432)
                    }
                }

                // 카메라 현재위치로 이동
                val cameraUpdate = CameraUpdate.scrollTo(
                    if(currentLocation != null && checkCamera){
                        LatLng(currentLocation!!.latitude, currentLocation!!.longitude)
                    }else{
                        checkCamera = true
                        LatLng(37.58253888738363,126.88582972882432)
                    }
                )
                naverMap.moveCamera(cameraUpdate)

                // 빨간색 마커 현재위치로 변경
                marker.position = LatLng(
                    naverMap.cameraPosition.target.latitude,
                    naverMap.cameraPosition.target.longitude
                )
            }
    }

    // 좌표 -> 주소 변환
    private fun getAddress(lat: Double, lng: Double): String {
        val geoCoder = Geocoder(requireContext(), Locale.KOREA)
        val address: ArrayList<Address>
        var addressResult = "주소를 가져 올 수 없습니다."
        try {
            //세번째 파라미터는 좌표에 대해 주소를 리턴 받는 갯수로
            //한좌표에 대해 두개이상의 이름이 존재할수있기에 주소배열을 리턴받기 위해 최대갯수 설정
            address = geoCoder.getFromLocation(lat, lng, 1) as ArrayList<Address>
            if (address.size > 0) {
                // 주소 받아오기
                val currentLocationAddress = address[0].getAddressLine(0)
                    .toString()
                addressResult = currentLocationAddress

            }

        } catch (e: IOException) {
            e.printStackTrace()
        }
        return addressResult
    }


    companion object {
        private const val LOCATION_PERMISSION_REQUEST_CODE = 1000
    }

}